//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "livox_lidar_callback.h"

#include "livox_lidar_api.h"
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "../include/livox_log.h"

namespace livox_ros {

// ---------------------------------------------------------------------------
// Race-condition analysis: dual-lidar initialisation
// ---------------------------------------------------------------------------
//
// RACE 1 — connect_state read/write without synchronisation (FIXED)
//   connect_state is written by SDK callback threads (inside config_mutex_) but
//   was previously read by the polling threads in lddc.cpp without any lock.
//   `volatile` prevents compiler optimisation but does NOT prevent CPU
//   reordering; a stale value could be observed indefinitely on some
//   architectures.  Fixed by changing connect_state to std::atomic<> in
//   comm.h, so every write is immediately visible to all readers.
//
// RACE 2 — LidarInfoChangeCallback re-fires while config is in flight (FIXED)
//   The Livox SDK re-fires LidarInfoChangeCallback whenever it re-discovers a
//   lidar (e.g. after a brief network hiccup or on a periodic poll).  Each
//   invocation ORs new bits into set_bits and dispatches fresh SDK commands to
//   the lidar.  If a re-fire happens before all ACKs from the first fire have
//   arrived, set_bits accumulates extra bits.  More critically, the fresh
//   SetLivoxLidarPclDataType / SetLivoxLidarScanPattern commands cause the
//   lidar to interrupt streaming while it applies the new config — and if any
//   of those second-round ACKs are lost, set_bits never reaches 0 and
//   connect_state never transitions to kConnectStateSampling, so no point
//   cloud data is ever published.
//
//   Fix: check connect_state at the start of the callback; if the lidar is
//   already Sampling, skip all config commands and only refresh work mode.
//   This prevents any re-fire from restarting the config handshake.
//
// RACE 3 — cross-instance SDK interference (PARTIALLY MITIGATED)
//   When dual_mid360_launch.py starts the second driver instance (after the
//   configured delay), that instance's Livox SDK initialises and discovers
//   ALL reachable lidars on the network — including the front lidar that
//   belongs to the first instance.  The application-level AllowHandle guard
//   prevents our *application* code from sending commands to foreign lidars,
//   but the Livox SDK itself may send lower-level discovery/handshake packets
//   (e.g. GetDeviceInfo) to all discovered devices.  These packets can cause
//   the front lidar to briefly re-negotiate its "host" endpoint, interrupting
//   the first driver's data stream.
//
//   Mitigations already in place:
//     * The back-lidar start delay (back_lidar_delay, default 5 s) lets the
//       front lidar fully handshake before the second SDK starts.
//     * The config JSON files specify explicit host IPs for each channel
//       (point_data_ip, push_msg_ip, etc.), which constrains which host the
//       lidar sends data to.
//     * AllowHandle filters packets at the PubHandler level.
//
//   Remaining exposure: if the SDK's internal discovery exchange changes the
//   lidar's "host" pointer, the only remedy is a longer delay or ensuring the
//   two config files use completely non-overlapping port ranges (already done:
//   front 56101-56105, back 56111-56115).  Increasing back_lidar_delay to 10 s
//   or more eliminates nearly all observed failures.
// ---------------------------------------------------------------------------

void LivoxLidarCallback::LidarInfoChangeCallback(const uint32_t handle,
                                           const LivoxLidarInfo* info,
                                           void* client_data) {
  if (client_data == nullptr) {
    LIVOX_ERROR("lidar info change callback failed, client data is nullptr");
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  LIVOX_INFO("[%s] LidarInfoChange event (handle=0x%08x)",
             IpNumToString(handle).c_str(), handle);

  LidarDevice* lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    // This lidar is not in this driver instance's user-defined config (it may
    // belong to another driver instance on the same host).  Do NOT send any
    // SDK commands (SetLivoxLidarWorkMode, EnableLivoxLidarImuData, etc.) to
    // it: doing so would make the lidar re-route its data stream to this
    // process, silently breaking the other driver instance that owns it.
    // Log once per foreign IP to avoid flooding the console on every
    // periodic SDK re-discovery event.
    static std::mutex s_foreign_mutex;
    static std::unordered_set<uint32_t> s_logged_foreign;
    std::lock_guard<std::mutex> lock(s_foreign_mutex);
    if (s_logged_foreign.insert(handle).second) {
      LIVOX_WARN("[%s] ignoring lidar — not in this instance's config"
                 " (likely owned by another driver process)",
                 IpNumToString(handle).c_str());
    }
    return;
  }

  // RACE 2 mitigation: if this lidar is already fully configured and
  // streaming, a re-fire of LidarInfoChangeCallback is a periodic SDK
  // re-discovery heartbeat.  Re-sending all config commands would interrupt
  // the active data stream.  Only refresh the work mode to keep streaming.
  if (lidar_device->connect_state.load(std::memory_order_acquire) == kConnectStateSampling) {
    LIVOX_INFO("[%s] LidarInfoChange re-fire (already Sampling),"
               " refreshing work mode only",
               IpNumToString(handle).c_str());
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr);
    return;
  }

  {
    // set the lidar according to the user-defined config
    const UserLivoxLidarConfig& config = lidar_device->livox_config;

    // lock for modify the lidar device set_bits
    {
      std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
      uint32_t pending_bits = 0;
      if (config.pcl_data_type != -1 ) {
        lidar_device->livox_config.set_bits |= kConfigDataType;
        pending_bits |= kConfigDataType;
        SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
                                LivoxLidarCallback::SetDataTypeCallback, lds_lidar);
        LIVOX_INFO("[%s] requesting pcl data type: %d",
                   IpNumToString(handle).c_str(), static_cast<int>(config.pcl_data_type));
      }
      if (config.pattern_mode != -1) {
        lidar_device->livox_config.set_bits |= kConfigScanPattern;
        pending_bits |= kConfigScanPattern;
        SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
                              LivoxLidarCallback::SetPatternModeCallback, lds_lidar);
        LIVOX_INFO("[%s] requesting scan pattern: %d",
                   IpNumToString(handle).c_str(), static_cast<int>(config.pattern_mode));
      }
      if (config.blind_spot_set != -1) {
        lidar_device->livox_config.set_bits |= kConfigBlindSpot;
        pending_bits |= kConfigBlindSpot;
        SetLivoxLidarBlindSpot(handle, config.blind_spot_set,
                              LivoxLidarCallback::SetBlindSpotCallback, lds_lidar);
        LIVOX_INFO("[%s] requesting blind spot: %d",
                   IpNumToString(handle).c_str(), config.blind_spot_set);
      }
      if (config.dual_emit_en != -1) {
        lidar_device->livox_config.set_bits |= kConfigDualEmit;
        pending_bits |= kConfigDualEmit;
        SetLivoxLidarDualEmit(handle, (config.dual_emit_en == 0 ? false : true),
                              LivoxLidarCallback::SetDualEmitCallback, lds_lidar);
        LIVOX_INFO("[%s] requesting dual emit: %d",
                   IpNumToString(handle).c_str(), static_cast<int>(config.dual_emit_en));
      }
      if (pending_bits == 0) {
        // No config commands needed — mark directly as sampling so data is forwarded.
        lidar_device->connect_state.store(kConnectStateSampling, std::memory_order_release);
        LIVOX_INFO("[%s] no config commands needed, state -> Sampling",
                   IpNumToString(handle).c_str());
      } else {
        // Count set bits using Brian Kernighan's algorithm (each iteration clears the lowest set bit).
        uint32_t pending_count = 0;
        for (uint32_t b = pending_bits; b != 0; b &= (b - 1)) { ++pending_count; }
        LIVOX_INFO("[%s] waiting for %u config ack(s) before Sampling (pending mask=0x%x)",
                   IpNumToString(handle).c_str(), pending_count, pending_bits);
      }
    } // free lock for set_bits

    // set extrinsic params into lidar
    LivoxLidarInstallAttitude attitude {
      config.extrinsic_param.roll,
      config.extrinsic_param.pitch,
      config.extrinsic_param.yaw,
      config.extrinsic_param.x,
      config.extrinsic_param.y,
      config.extrinsic_param.z
    };
    SetLivoxLidarInstallAttitude(config.handle, &attitude,
                                 LivoxLidarCallback::SetAttitudeCallback, lds_lidar);
  }

  LIVOX_INFO("[%s] requesting work mode -> Normal", IpNumToString(handle).c_str());
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr);
  EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar);
  return;
}

void LivoxLidarCallback::WorkModeChangedCallback(livox_status status,
                                                 uint32_t handle,
                                                 LivoxLidarAsyncControlResponse *response,
                                                 void *client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    // Track per-handle retry count to surface persistent failures.
    static std::mutex s_retry_mutex;
    static std::unordered_map<uint32_t, int> s_retry_count;
    int retries;
    {
      std::lock_guard<std::mutex> lock(s_retry_mutex);
      retries = ++s_retry_count[handle];
    }
    LIVOX_WARN("[%s] work mode change failed (status=%d), retry #%d",
               IpNumToString(handle).c_str(), static_cast<int>(status), retries);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr);
    return;
  }
  LIVOX_INFO("[%s] work mode -> Normal: OK", IpNumToString(handle).c_str());
  return;
}

void LivoxLidarCallback::SetDataTypeCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_ERROR("[%s] set data type ack: lidar device not found",
                IpNumToString(handle).c_str());
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigDataType));
    LIVOX_INFO("[%s] data type set OK, remaining config bits: %u",
               IpNumToString(handle).c_str(), lidar_device->livox_config.set_bits);
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state.store(kConnectStateSampling, std::memory_order_release);
      LIVOX_INFO("[%s] all config acks received — state -> Sampling, data will be published",
                 IpNumToString(handle).c_str());
    }
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
                             LivoxLidarCallback::SetDataTypeCallback, client_data);
    LIVOX_WARN("[%s] set data type timed out, retrying...", IpNumToString(handle).c_str());
  } else {
    if (response) {
      LIVOX_ERROR("[%s] set data type failed: ret_code=%d error_key=%d",
                  IpNumToString(handle).c_str(), response->ret_code, response->error_key);
    } else {
      LIVOX_ERROR("[%s] set data type failed: status=%d (no response payload)",
                  IpNumToString(handle).c_str(), static_cast<int>(status));
    }
  }
  return;
}

void LivoxLidarCallback::SetPatternModeCallback(livox_status status, uint32_t handle,
                                                LivoxLidarAsyncControlResponse *response,
                                                void *client_data) {
  LidarDevice* lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_ERROR("[%s] set pattern mode ack: lidar device not found",
                IpNumToString(handle).c_str());
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigScanPattern));
    LIVOX_INFO("[%s] scan pattern set OK, remaining config bits: %u",
               IpNumToString(handle).c_str(), lidar_device->livox_config.set_bits);
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state.store(kConnectStateSampling, std::memory_order_release);
      LIVOX_INFO("[%s] all config acks received — state -> Sampling, data will be published",
                 IpNumToString(handle).c_str());
    }
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
                             LivoxLidarCallback::SetPatternModeCallback, client_data);
    LIVOX_WARN("[%s] set scan pattern timed out, retrying...", IpNumToString(handle).c_str());
  } else {
    if (response) {
      LIVOX_ERROR("[%s] set scan pattern failed: ret_code=%d error_key=%d",
                  IpNumToString(handle).c_str(), response->ret_code, response->error_key);
    } else {
      LIVOX_ERROR("[%s] set scan pattern failed: status=%d (no response payload)",
                  IpNumToString(handle).c_str(), static_cast<int>(status));
    }
  }
  return;
}

void LivoxLidarCallback::SetBlindSpotCallback(livox_status status, uint32_t handle,
                                              LivoxLidarAsyncControlResponse *response,
                                              void *client_data) {
  LidarDevice* lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_ERROR("[%s] set blind spot ack: lidar device not found",
                IpNumToString(handle).c_str());
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigBlindSpot));
    LIVOX_INFO("[%s] blind spot set OK, remaining config bits: %u",
               IpNumToString(handle).c_str(), lidar_device->livox_config.set_bits);
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state.store(kConnectStateSampling, std::memory_order_release);
      LIVOX_INFO("[%s] all config acks received — state -> Sampling, data will be published",
                 IpNumToString(handle).c_str());
    }
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarBlindSpot(handle, config.blind_spot_set,
                           LivoxLidarCallback::SetBlindSpotCallback, client_data);
    LIVOX_WARN("[%s] set blind spot timed out, retrying...", IpNumToString(handle).c_str());
  } else {
    if (response) {
      LIVOX_ERROR("[%s] set blind spot failed: ret_code=%d error_key=%d",
                  IpNumToString(handle).c_str(), response->ret_code, response->error_key);
    } else {
      LIVOX_ERROR("[%s] set blind spot failed: status=%d (no response payload)",
                  IpNumToString(handle).c_str(), static_cast<int>(status));
    }
  }
  return;
}

void LivoxLidarCallback::SetDualEmitCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_ERROR("[%s] set dual emit ack: lidar device not found",
                IpNumToString(handle).c_str());
    return;
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigDualEmit));
    LIVOX_INFO("[%s] dual emit set OK, remaining config bits: %u",
               IpNumToString(handle).c_str(), lidar_device->livox_config.set_bits);
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state.store(kConnectStateSampling, std::memory_order_release);
      LIVOX_INFO("[%s] all config acks received — state -> Sampling, data will be published",
                 IpNumToString(handle).c_str());
    }
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarDualEmit(handle, config.dual_emit_en,
                          LivoxLidarCallback::SetDualEmitCallback, client_data);
    LIVOX_WARN("[%s] set dual emit timed out, retrying...", IpNumToString(handle).c_str());
  } else {
    if (response) {
      LIVOX_ERROR("[%s] set dual emit failed: ret_code=%d error_key=%d",
                  IpNumToString(handle).c_str(), response->ret_code, response->error_key);
    } else {
      LIVOX_ERROR("[%s] set dual emit failed: status=%d (no response payload)",
                  IpNumToString(handle).c_str(), static_cast<int>(status));
    }
  }
  return;
}

void LivoxLidarCallback::SetAttitudeCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_ERROR("[%s] set attitude ack: lidar device not found",
                IpNumToString(handle).c_str());
    return;
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  if (status == kLivoxLidarStatusSuccess) {
    LIVOX_INFO("[%s] install attitude set OK", IpNumToString(handle).c_str());
  } else if (status == kLivoxLidarStatusTimeout) {
    LIVOX_WARN("[%s] set attitude timed out, retrying...", IpNumToString(handle).c_str());
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    LivoxLidarInstallAttitude attitude {
      config.extrinsic_param.roll,
      config.extrinsic_param.pitch,
      config.extrinsic_param.yaw,
      config.extrinsic_param.x,
      config.extrinsic_param.y,
      config.extrinsic_param.z
    };
    SetLivoxLidarInstallAttitude(config.handle, &attitude,
                                 LivoxLidarCallback::SetAttitudeCallback, lds_lidar);
  } else {
    LIVOX_ERROR("[%s] set attitude failed (status=%d)",
                IpNumToString(handle).c_str(), static_cast<int>(status));
  }
}

void LivoxLidarCallback::EnableLivoxLidarImuDataCallback(livox_status status, uint32_t handle,
                                                         LivoxLidarAsyncControlResponse *response,
                                                         void *client_data) {
  LidarDevice* lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_ERROR("[%s] enable IMU ack: lidar device not found", IpNumToString(handle).c_str());
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (response == nullptr) {
    LIVOX_WARN("[%s] enable IMU: no response received", IpNumToString(handle).c_str());
    return;
  }

  if (status == kLivoxLidarStatusSuccess) {
    LIVOX_INFO("[%s] IMU data enabled OK", IpNumToString(handle).c_str());
  } else if (status == kLivoxLidarStatusTimeout) {
    LIVOX_WARN("[%s] enable IMU timed out, retrying...", IpNumToString(handle).c_str());
    EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar);
  } else {
    LIVOX_ERROR("[%s] enable IMU failed (status=%d)",
                IpNumToString(handle).c_str(), static_cast<int>(status));
  }
}

LidarDevice* LivoxLidarCallback::GetLidarDevice(const uint32_t handle, void* client_data) {
  if (client_data == nullptr) {
    LIVOX_ERROR("GetLidarDevice: client data is nullptr");
    return nullptr;
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  uint8_t index = 0;
  int8_t ret = lds_lidar->cache_index_.GetIndex(kLivoxLidarType, handle, index);
  if (ret != 0) {
    return nullptr;
  }

  return &(lds_lidar->lidars_[index]);
}

} // namespace livox_ros

