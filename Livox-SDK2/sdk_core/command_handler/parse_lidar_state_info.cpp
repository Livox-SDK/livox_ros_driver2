
#include "parse_lidar_state_info.h"
#include "base/logging.h"

#include <iostream>
#include <sstream>

namespace livox {

namespace lidar {

bool ParseLidarStateInfo::Parse(const CommPacket& packet, std::string& info_str) {
  DirectLidarStateInfo info;
  std::set<ParamKeyName> key_mask;
  
  if (!ParseStateInfo(packet, info, key_mask)) {
    return false;
  }

  LivoxLidarStateInfoToJson(info, key_mask, info_str);
  return true;
}

bool ParseLidarStateInfo::ParseStateInfo(const CommPacket& packet,
                                         DirectLidarStateInfo& info,
                                         std::set<ParamKeyName>& key_mask) {  
  uint16_t offset = 0;
  uint16_t key_num = 0;
  memcpy(&key_num, &packet.data[offset], sizeof(uint16_t));
  offset += sizeof(uint16_t) * 2;  

  for (uint16_t i = 0; i < key_num; ++i) {
    if (offset + sizeof(LivoxLidarKeyValueParam) > packet.data_len) {
      return false;
    }

    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&packet.data[offset];
    offset += sizeof(uint16_t);

    uint16_t val_len = 0;
    memcpy(&val_len, &packet.data[offset], sizeof(uint16_t));
    offset += sizeof(uint16_t);
  
    switch (kv->key) {
      case static_cast<uint16_t>(kKeyPclDataType) :
        key_mask.insert(kKeyPclDataType);
        memcpy(&info.pcl_data_type, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyPatternMode) :
        key_mask.insert(kKeyPatternMode);
        memcpy(&info.pattern_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyDualEmitEn) :
        key_mask.insert(kKeyDualEmitEn);
        memcpy(&info.dual_emit_en, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyPointSendEn) :
        key_mask.insert(kKeyPointSendEn);
        memcpy(&info.point_send_en, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyLidarIpCfg) :
        key_mask.insert(kKeyLidarIpCfg);
        ParseLidarIpAddr(packet, offset, info);
        break;
      case static_cast<uint16_t>(kKeyStateInfoHostIpCfg) :
        key_mask.insert(kKeyStateInfoHostIpCfg);
        ParseStateInfoHostIPCfg(packet, offset, info);
        break;
      case static_cast<uint16_t>(kKeyLidarPointDataHostIpCfg) :
        key_mask.insert(kKeyLidarPointDataHostIpCfg);
        ParsePointCloudHostIpCfg(packet, offset, info);
        break;
      case static_cast<uint16_t>(kKeyLidarImuHostIpCfg) :
        key_mask.insert(kKeyLidarImuHostIpCfg);
        ParseImuDataHostIpCfg(packet, offset, info);
        break;
      case static_cast<uint16_t>(kKeyCtlHostIpCfg) :
        key_mask.insert(kKeyCtlHostIpCfg);
        ParseIpCfg(packet, offset, info.ctl_host_ipcfg);
        break;
      case static_cast<uint16_t>(kKeyLogHostIpCfg) :
        key_mask.insert(kKeyLogHostIpCfg);
        ParseIpCfg(packet, offset, info.log_host_ipcfg);
        break;
      case static_cast<uint16_t>(kKeyVehicleSpeed) :
        key_mask.insert(kKeyVehicleSpeed);
        memcpy(&info.vehicle_speed, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyEnvironmentTemp) :
        key_mask.insert(kKeyEnvironmentTemp);
        memcpy(&info.environment_temp, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyInstallAttitude) :
        key_mask.insert(kKeyInstallAttitude);
        memcpy(&info.install_attitude, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyBlindSpotSet) :
        key_mask.insert(kKeyBlindSpotSet);
        memcpy(&info.blind_spot_set, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFrameRate) :
        key_mask.insert(kKeyFrameRate);
        memcpy(&info.frame_rate, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFovCfg0) :
        key_mask.insert(kKeyFovCfg0);
        memcpy(&info.fov_cfg0, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFovCfg1) :
        key_mask.insert(kKeyFovCfg1);
        memcpy(&info.fov_cfg1, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFovCfgEn) :
        key_mask.insert(kKeyFovCfgEn);
        memcpy(&info.fov_cfg_en, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyDetectMode) :
        key_mask.insert(kKeyDetectMode);
        memcpy(&info.detect_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFuncIoCfg) :
        key_mask.insert(kKeyFuncIoCfg);
        memcpy(&info.func_io_cfg, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyWorkMode) :
        key_mask.insert(kKeyWorkMode);
        memcpy(&info.work_tgt_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyGlassHeat) :
        key_mask.insert(kKeyGlassHeat);
        memcpy(&info.glass_heat, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyImuDataEn) :
        key_mask.insert(kKeyImuDataEn);
        memcpy(&info.imu_data_en, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFusaEn) :
        key_mask.insert(kKeyFusaEn);
        memcpy(&info.fusa_en, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySn) :
        key_mask.insert(kKeySn);
        memcpy(info.sn, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyProductInfo) :
        key_mask.insert(kKeyProductInfo);
        memcpy(info.product_info, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyVersionApp) :
        key_mask.insert(kKeyVersionApp);
        memcpy(info.version_app, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyVersionLoader) :
        key_mask.insert(kKeyVersionLoader);
        memcpy(info.version_loader, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyVersionHardware):
        key_mask.insert(kKeyVersionHardware);
        memcpy(info.version_hardware, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyMac) :
        key_mask.insert(kKeyMac);
        memcpy(info.mac, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyCurWorkState) :
        key_mask.insert(kKeyCurWorkState);
        memcpy(&info.cur_work_state, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyCoreTemp) :
        key_mask.insert(kKeyCoreTemp);
        memcpy(&info.core_temp, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyPowerUpCnt) :
        key_mask.insert(kKeyPowerUpCnt);
        memcpy(&info.powerup_cnt, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyLocalTimeNow) :
        key_mask.insert(kKeyLocalTimeNow);
        memcpy(&info.local_time_now, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyLastSyncTime) :
        key_mask.insert(kKeyLastSyncTime);
        memcpy(&info.last_sync_time, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyTimeOffset) :
        key_mask.insert(kKeyTimeOffset);
        memcpy(&info.time_offset, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyTimeSyncType) :
        key_mask.insert(kKeyTimeSyncType);
        memcpy(&info.time_sync_type, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyStatusCode) :
        key_mask.insert(kKeyStatusCode);
        memcpy(&info.status_code, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyLidarDiagStatus) :
        key_mask.insert(kKeyLidarDiagStatus);
        memcpy(&info.lidar_diag_status, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyLidarFlashStatus) :
        key_mask.insert(kKeyLidarFlashStatus);
        memcpy(&info.lidar_flash_status, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFwType) :
        key_mask.insert(kKeyFwType);
        memcpy(&info.fw_type, &packet.data[offset], val_len);
        break; 
      case static_cast<uint16_t>(kKeyHmsCode) :
        key_mask.insert(kKeyHmsCode);
        memcpy(&info.hms_code, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyRoiMode) :
        key_mask.insert(kKeyRoiMode);
        memcpy(&info.ROI_Mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetEscMode) :
        key_mask.insert(kKeySetEscMode);
        memcpy(&info.esc_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetFovMode) :
        key_mask.insert(kKeySetFovMode);
        memcpy(&info.fov_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetEchoMode) :
        key_mask.insert(kKeySetEchoMode);
        memcpy(&info.echo_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetNTPServerIp) : {
        key_mask.insert(kKeySetNTPServerIp);
        uint8_t ip[4];
        memcpy(ip, &packet.data[offset], sizeof(uint8_t) * 4);
        std::string ip_str = std::to_string(ip[0]) + "." + std::to_string(ip[1]) + "." +
            std::to_string(ip[2]) + "." + std::to_string(ip[3]);
        strcpy(info.ntp_server_ip.host_ip, ip_str.c_str());
        break;
      }
      case static_cast<uint16_t>(kKeySetITOCtrl) :
        key_mask.insert(kKeySetITOCtrl);
        memcpy(&info.ito_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetFogNoiseFilter) :
        key_mask.insert(kKeySetFogNoiseFilter);
        memcpy(&info.fog_noise_filter, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetTimeFilterMode) :
        key_mask.insert(kKeySetTimeFilterMode);
        memcpy(&info.time_filter_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetPclFreqMod) :
        key_mask.insert(kKeySetPclFreqMod);
        memcpy(&info.pcl_freq_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetImuRange) :
        key_mask.insert(kKeySetImuRange);
        memcpy(&info.imu_range, &packet.data[offset], val_len);
        break;
      default :
        break;
    }
    offset += val_len;
  }

  // printf("Lidar state info, pcl_data_type:%d, pattern_mode:%d, lidar_ip:%s, lidar_submask:%s, lidar_gatway:%s.\n",
  //     info.pcl_data_type, info.pattern_mode, info.lidar_ip_info.ip_addr, info.lidar_ip_info.net_mask, 
  //     info.lidar_ip_info.gw_addr);
  
  // printf("Lidar state info, host_ip_addr:%s, host_state_info_port:%u, lidar_state_info_port:%u.\n",
  //     info.host_state_info.host_ip_addr, info.host_state_info.host_state_info_port, info.host_state_info.lidar_state_info_port);

  // printf("Lidar state info, host_ip_addr:%s, host_point_data_port:%u, lidar_point_data_port:%u.\n",
  //     info.pointcloud_host_ipcfg.host_ip_addr, info.pointcloud_host_ipcfg.host_point_data_port,
  //     info.pointcloud_host_ipcfg.lidar_point_data_port);
    
  // printf("Lidar state info, host_ip_addr:%s, host_imu_data_port:%u, lidar_imu_data_port:%u.\n",
  //     info.imu_host_ipcfg.host_ip_addr, info.pointcloud_host_ipcfg.host_point_data_port,
  //     info.imu_host_ipcfg.lidar_imu_data_port);

  // printf("Lidar state info, roll:%f, pitch:%f, yaw:%f, x:%d, y:%d,z:%d.\n",
  //        info.install_attitude.roll_deg, info.install_attitude.pitch_deg,
  //        info.install_attitude.yaw_deg, info.install_attitude.x, info.install_attitude.y, info.install_attitude.z);
  
  // printf("Lidar state info, fov cfg0, yaw_start:%d, yaw_stop:%d, pitch_start:%d, pitch_stop:%d.\n",
  //     info.fov_cfg0.yaw_start, info.fov_cfg0.yaw_stop, info.fov_cfg0.pitch_start, info.fov_cfg0.pitch_stop);

  // printf("Lidar state info, fov cfg1, yaw_start:%d, yaw_stop:%d, pitch_start:%d, pitch_stop:%d.\n",
  //     info.fov_cfg1.yaw_start, info.fov_cfg1.yaw_stop, info.fov_cfg1.pitch_start, info.fov_cfg1.pitch_stop);

  // printf("Lidar state info, fov_en:%u, work_mode:%u, imu_data_en:%u, sn:%s, product_info:%s.\n",
  //     info.fov_en, info.work_mode, info.imu_data_en, info.sn, info.product_info);

  // std::string version_app = std::to_string(info.version_app[0]) + ":" + std::to_string(info.version_app[1]) + ":" + 
  //     std::to_string(info.version_app[2]) + ":" + std::to_string(info.version_app[3]);

  // std::string version_load = std::to_string(info.version_load[0]) + ":" + std::to_string(info.version_load[1]) + ":" + 
  //     std::to_string(info.version_load[2]) + ":" + std::to_string(info.version_load[3]);

  // std::string version_hardware = std::to_string(info.version_hardware[0]) + ":" + std::to_string(info.version_hardware[1]) + ":" + 
  //     std::to_string(info.version_hardware[2]) + ":" + std::to_string(info.version_hardware[3]);
  
  // std::string mac = std::to_string(info.mac[0]) + ":" + std::to_string(info.mac[1]) + ":" + 
  //     std::to_string(info.mac[2]) + ":" + std::to_string(info.mac[3]) + ":" +
  //     std::to_string(info.mac[4]) + ":" + std::to_string(info.mac[5]);

  // printf("Lidar state info, version_app:%s, version_load:%s, version_hardware:%s, mac:%s.\n",
  //     version_app.c_str(), version_load.c_str(), version_hardware.c_str(), mac.c_str());


  // printf("Lidar state info, cur_work_state:%u, core_temp:%d, powerup_cnt:%u, local_time_now:%lu, last_sync_time:%lu, time_offset:%ld.\n",
  //     info.cur_work_state, info.core_temp, info.powerup_cnt, info.local_time_now, info.last_sync_time, info.time_offset);
  
  // printf("Lidar state info, time_sync_type:%u, fw_type:%u.\n", info.time_sync_type, info.fw_type);

  return true;
}

void ParseLidarStateInfo::ParseLidarIpAddr(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info) {
  uint8_t lidar_ip[4];
  memcpy(lidar_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_ip_str = std::to_string(lidar_ip[0]) + "." + std::to_string(lidar_ip[1]) + "." + 
      std::to_string(lidar_ip[2]) + "." + std::to_string(lidar_ip[3]);
  strcpy(info.lidar_ipcfg.ip_addr, lidar_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  uint8_t lidar_submask[4];
  memcpy(lidar_submask, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_submask_str = std::to_string(lidar_submask[0]) + "." + std::to_string(lidar_submask[1]) + 
      "." + std::to_string(lidar_submask[2]) + "." + std::to_string(lidar_submask[3]);
  strcpy(info.lidar_ipcfg.net_mask, lidar_submask_str.c_str());
  off += sizeof(uint8_t) * 4;
  
  uint8_t lidar_gateway[4];
  memcpy(lidar_gateway, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_gateway_str = std::to_string(lidar_gateway[0]) + "." + std::to_string(lidar_gateway[1]) +
      "." + std::to_string(lidar_gateway[2]) + "." + std::to_string(lidar_gateway[3]);
  strcpy(info.lidar_ipcfg.gw_addr, lidar_gateway_str.c_str());
}

void ParseLidarStateInfo::ParseStateInfoHostIPCfg(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info) {
  uint8_t host_state_info_ip[4];
  memcpy(host_state_info_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_state_info_ip_str = std::to_string(host_state_info_ip[0]) + "." + 
      std::to_string(host_state_info_ip[1]) + "." + std::to_string(host_state_info_ip[2]) + "." +
      std::to_string(host_state_info_ip[3]);
  
  strcpy(info.host_state_info.host_ip_addr, host_state_info_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.host_state_info.host_state_info_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);

  memcpy(&info.host_state_info.lidar_state_info_port, &packet.data[off], sizeof(uint16_t));
}

void ParseLidarStateInfo::ParsePointCloudHostIpCfg(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info) {
  uint8_t host_point_cloud_ip[4];
  memcpy(host_point_cloud_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_point_cloud_ip_str = std::to_string(host_point_cloud_ip[0]) + "." + 
      std::to_string(host_point_cloud_ip[1]) + "." + std::to_string(host_point_cloud_ip[2]) + "." +
      std::to_string(host_point_cloud_ip[3]);
  
  strcpy(info.pointcloud_host_ipcfg.host_ip_addr, host_point_cloud_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.pointcloud_host_ipcfg.host_point_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);

  memcpy(&info.pointcloud_host_ipcfg.lidar_point_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);
}

void ParseLidarStateInfo::ParseImuDataHostIpCfg(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info) {
  uint8_t host_imu_data_ip[4];
  memcpy(host_imu_data_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_imu_data_ip_str = std::to_string(host_imu_data_ip[0]) + "." + 
      std::to_string(host_imu_data_ip[1]) + "." + std::to_string(host_imu_data_ip[2]) + "." +
      std::to_string(host_imu_data_ip[3]);
  
  strcpy(info.imu_host_ipcfg.host_ip_addr, host_imu_data_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.imu_host_ipcfg.host_imu_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);

  memcpy(&info.imu_host_ipcfg.lidar_imu_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);
}

void ParseLidarStateInfo::ParseIpCfg(const CommPacket& packet, uint16_t off, LivoxIpCfg& cfg) {
  std::string ip_str = std::to_string(packet.data[off]) + "." + 
                       std::to_string(packet.data[off + 1]) + "." + 
                       std::to_string(packet.data[off + 2]) + "." +
                       std::to_string(packet.data[off + 3]);
  strcpy(cfg.ip_addr, ip_str.c_str());
  off += sizeof(uint8_t) * 4;
  cfg.dst_port = *(uint16_t*)&packet.data[off];
  off += sizeof(uint16_t); 
  cfg.src_port = *(uint16_t*)&packet.data[off];
  return;
}

void ParseLidarStateInfo::LivoxLidarStateInfoToJson(const DirectLidarStateInfo& info, const std::set<ParamKeyName>& key_mask, std::string& lidar_info) {
  rapidjson::StringBuffer buf;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> write(buf);
  write.StartObject();

  // write.Key("dev_type");
  // write.String("MID360");
  
  if (key_mask.find(kKeyPclDataType) != key_mask.end()) {
    write.Key("pcl_data_type");
    write.Uint(info.pcl_data_type);    
  }

  if (key_mask.find(kKeyPatternMode) != key_mask.end()) {
    write.Key("pattern_mode");
    write.Uint(info.pattern_mode);
  }
  
  if (key_mask.find(kKeyDualEmitEn) != key_mask.end()) {
    write.Key("dual_emit_en");
    write.Uint(info.dual_emit_en);
  }
  
  if (key_mask.find(kKeyPointSendEn) != key_mask.end()) {
    write.Key("point_send_en");
    write.Uint(info.point_send_en);
  }
  
  if (key_mask.find(kKeyLidarIpCfg) != key_mask.end()) {
    write.Key("lidar_ipcfg");
    write.StartObject();
    write.Key("lidar_ip");
    write.String(info.lidar_ipcfg.ip_addr);
    write.Key("lidar_subnet_mask");
    write.String(info.lidar_ipcfg.net_mask);
    write.Key("lidar_gateway");
    write.String(info.lidar_ipcfg.gw_addr);
    write.EndObject();    
  }
  
  if (key_mask.find(kKeyStateInfoHostIpCfg) != key_mask.end()) {
    write.Key("state_info_host_ipcfg");
    write.StartObject();
    write.Key("ip");
    write.String(info.host_state_info.host_ip_addr);
    write.Key("dst_port");
    write.Uint(info.host_state_info.host_state_info_port);
    write.Key("src_port");
    write.Uint(info.host_state_info.lidar_state_info_port);
    write.EndObject();
  }
  
  if (key_mask.find(kKeyLidarPointDataHostIpCfg) != key_mask.end()) {
    write.Key("ponitcloud_host_ipcfg");
    write.StartObject();
    write.Key("ip");
    write.String(info.pointcloud_host_ipcfg.host_ip_addr);
    write.Key("dst_port");
    write.Uint(info.pointcloud_host_ipcfg.host_point_data_port);
    write.Key("src_port");
    write.Uint(info.pointcloud_host_ipcfg.lidar_point_data_port);
    write.EndObject();
  }
  
  if (key_mask.find(kKeyLidarImuHostIpCfg) != key_mask.end()) {
    write.Key("imu_host_ipcfg");
    write.StartObject();
    write.Key("ip");
    write.String(info.imu_host_ipcfg.host_ip_addr);
    write.Key("dst_port");
    write.Uint(info.imu_host_ipcfg.host_imu_data_port);
    write.Key("src_port");
    write.Uint(info.imu_host_ipcfg.lidar_imu_data_port);
    write.EndObject();
  }
  
  if (key_mask.find(kKeyCtlHostIpCfg) != key_mask.end()) {
    write.Key("ctl_host_ipcfg");
    write.StartObject();
    write.Key("ip");
    write.String(info.ctl_host_ipcfg.ip_addr);
    write.Key("dst_port");
    write.Uint(info.ctl_host_ipcfg.dst_port);
    write.Key("src_port");
    write.Uint(info.ctl_host_ipcfg.src_port);
    write.EndObject();
  }
  
  if (key_mask.find(kKeyLogHostIpCfg) != key_mask.end()) {
    write.Key("log_host_ipcfg");
    write.StartObject();
    write.Key("ip");
    write.String(info.log_host_ipcfg.ip_addr);
    write.Key("dst_port");
    write.Uint(info.log_host_ipcfg.dst_port);
    write.Key("src_port");
    write.Uint(info.log_host_ipcfg.src_port);
    write.EndObject();
  }
  
  if (key_mask.find(kKeyVehicleSpeed) != key_mask.end()) {
    write.Key("vehicle_speed");
    write.Int(info.vehicle_speed);
  }
  
  if (key_mask.find(kKeyEnvironmentTemp) != key_mask.end()) {
    write.Key("environment_temp");
    write.Int(info.environment_temp);
  }
  
  if (key_mask.find(kKeyInstallAttitude) != key_mask.end()) {
    write.Key("install_attitude");
    write.StartObject();
    write.Key("roll_deg");
    write.Double(info.install_attitude.roll_deg);
    write.Key("pitch_deg");
    write.Double(info.install_attitude.pitch_deg);
    write.Key("yaw_deg");
    write.Double(info.install_attitude.yaw_deg);
    write.Key("x_mm");
    write.Uint(info.install_attitude.x);
    write.Key("y_mm");
    write.Uint(info.install_attitude.y);
    write.Key("z_mm");
    write.Uint(info.install_attitude.z);
    write.EndObject();
  }
  
  if (key_mask.find(kKeyBlindSpotSet) != key_mask.end()) {
    write.Key("blind_spot_set");
    write.Uint(info.blind_spot_set);
  }
  
  if (key_mask.find(kKeyFrameRate) != key_mask.end()) {
    write.Key("frame_rate");
    write.Uint(info.frame_rate);
  }
  
  if (key_mask.find(kKeyFovCfg0) != key_mask.end()) {
    write.Key("fov_cfg0");
    write.StartObject();
    write.Key("yaw_start");
    write.Int(info.fov_cfg0.yaw_start);
    write.Key("yaw_stop");
    write.Int(info.fov_cfg0.yaw_stop);
    write.Key("pitch_start");
    write.Int(info.fov_cfg0.pitch_start);
    write.Key("pitch_stop");
    write.Int(info.fov_cfg0.pitch_stop);
    write.EndObject();
  }
  
  if (key_mask.find(kKeyFovCfg1) != key_mask.end()) {
    write.Key("fov_cfg1");
    write.StartObject();
    write.Key("yaw_start");
    write.Int(info.fov_cfg1.yaw_start);
    write.Key("yaw_stop");
    write.Int(info.fov_cfg1.yaw_stop);
    write.Key("pitch_start");
    write.Int(info.fov_cfg1.pitch_start);
    write.Key("pitch_stop");
    write.Int(info.fov_cfg1.pitch_stop);
    write.EndObject(); 
  }
  
  if (key_mask.find(kKeyFovCfgEn) != key_mask.end()) {
    write.Key("fov_cfg_en");
    write.Uint(info.fov_cfg_en);
  }
  
  if (key_mask.find(kKeyDetectMode) != key_mask.end()) {
    write.Key("detect_mode");
    write.Uint(info.detect_mode);
  }
  
  if (key_mask.find(kKeyFuncIoCfg) != key_mask.end()) {
    write.Key("func_io_cfg");
    write.StartObject();
    write.Key("IN0");
    write.Uint(info.func_io_cfg[0]);
    write.Key("IN1");
    write.Uint(info.func_io_cfg[1]);
    write.Key("OUT0");
    write.Uint(info.func_io_cfg[2]);
    write.Key("OUT1");
    write.Uint(info.func_io_cfg[3]);
    write.EndObject(); 
  }

  if (key_mask.find(kKeyWorkMode) != key_mask.end()) {
    write.Key("work_tgt_mode");
    write.Uint(info.work_tgt_mode);
  }
  
  if (key_mask.find(kKeyGlassHeat) != key_mask.end()) {
    write.Key("glass_heat");
    write.Uint(info.glass_heat);
  }

  if (key_mask.find(kKeyImuDataEn) != key_mask.end()) {
    write.Key("imu_data_en");
    write.Uint(info.imu_data_en);
  }
  
  if (key_mask.find(kKeyFusaEn) != key_mask.end()) {
    write.Key("fusa_en");
    write.Uint(info.fusa_en);
  }

  if (key_mask.find(kKeySetEscMode) != key_mask.end()) {
    write.Key("esc_mode");
    write.Uint(info.esc_mode);
  }

  if (key_mask.find(kKeySetFovMode) != key_mask.end()) {
    write.Key("fov_mode");
    write.Uint(info.fov_mode);
  }

  if (key_mask.find(kKeySetEchoMode) != key_mask.end()) {
    write.Key("echo_mode");
    write.Uint(info.echo_mode);
  }

  if (key_mask.find(kKeySetNTPServerIp) != key_mask.end()) {
    write.Key("ntp_server_ip");
    write.String(info.ntp_server_ip.host_ip);
  }

  if (key_mask.find(kKeySetITOCtrl) != key_mask.end()) {
    write.Key("ito_mode");
    write.Uint(info.ito_mode);
  }

  if (key_mask.find(kKeySetFogNoiseFilter) != key_mask.end()) {
    write.Key("fog_noise_filter");
    write.Uint(info.fog_noise_filter);
  }

  if (key_mask.find(kKeySetPclFreqMod) != key_mask.end()) {
    write.Key("pcl_freq_mode");
    write.Uint(info.pcl_freq_mode);
  }

  if (key_mask.find(kKeySetTimeFilterMode) != key_mask.end()) {
    write.Key("time_filter_mode");
    write.Uint(info.time_filter_mode);
  }

  if (key_mask.find(kKeySetImuRange) != key_mask.end()) {
    write.Key("imu_range");
    write.StartObject();
    write.Key("imu_out_rate");
    write.Uint(info.imu_range.imu_out_rate);
    write.Key("accel_range");
    write.Uint(info.imu_range.accel_range);
    write.Key("gyro_range");
    write.Uint(info.imu_range.gyro_range);
    write.EndObject();
  }
  
  if (key_mask.find(kKeySn) != key_mask.end()) {
    write.Key("sn");
    write.String(info.sn);
  }
  
  if (key_mask.find(kKeyProductInfo) != key_mask.end()) {
    write.Key("product_info");
    write.String(info.product_info);
  }
  
  if (key_mask.find(kKeyVersionApp) != key_mask.end()) {
    write.Key("version_app");
    write.StartArray();
    write.Uint(info.version_app[0]);
    write.Uint(info.version_app[1]);
    write.Uint(info.version_app[2]);
    write.Uint(info.version_app[3]);
    write.EndArray();
  }
  
  if (key_mask.find(kKeyVersionLoader) != key_mask.end()) {
    write.Key("version_loader");
    write.StartArray();
    write.Uint(info.version_loader[0]);
    write.Uint(info.version_loader[1]);
    write.Uint(info.version_loader[2]);
    write.Uint(info.version_loader[3]);
    write.EndArray();
  }
  
  if (key_mask.find(kKeyVersionHardware) != key_mask.end()) {
    write.Key("version_hardware");
    write.StartArray();
    write.Uint(info.version_hardware[0]);
    write.Uint(info.version_hardware[1]);
    write.Uint(info.version_hardware[2]);
    write.Uint(info.version_hardware[3]);
    write.EndArray();
  }
  
  if (key_mask.find(kKeyMac) != key_mask.end()) {
    write.Key("mac");
    write.StartArray();
    write.Uint(info.mac[0]);
    write.Uint(info.mac[1]);
    write.Uint(info.mac[2]);
    write.Uint(info.mac[3]);
    write.Uint(info.mac[4]);
    write.Uint(info.mac[5]);
    write.EndArray();
  }
  
  if (key_mask.find(kKeyCurWorkState) != key_mask.end()) {
    write.Key("cur_work_state");
    write.Uint(info.cur_work_state);
  }
  
  if (key_mask.find(kKeyCoreTemp) != key_mask.end()) {
    write.Key("core_temp");
    write.Int(info.core_temp);
  }
  
  if (key_mask.find(kKeyPowerUpCnt) != key_mask.end()) {
    write.Key("powerup_cnt");
    write.Uint(info.powerup_cnt);
  }
  
  if (key_mask.find(kKeyLocalTimeNow) != key_mask.end()) {
    write.Key("local_time_now");
    write.Uint64(info.local_time_now);
  }
  
  if (key_mask.find(kKeyLastSyncTime) != key_mask.end()) {
    write.Key("last_sync_time");
    write.Uint64(info.last_sync_time);
  }
  
  if (key_mask.find(kKeyTimeOffset) != key_mask.end()) {
    write.Key("time_offset");
    write.Int64(info.time_offset);
  }
  
  if (key_mask.find(kKeyTimeSyncType) != key_mask.end()) {
    write.Key("time_sync_type");
    write.Uint(info.time_sync_type);
  }
  
  if (key_mask.find(kKeyStatusCode) != key_mask.end()) {
    write.Key("status_code");
    std::ostringstream ss;
    for (int idx = 31; idx >= 0; --idx) {
      ss << std::hex << static_cast<uint32_t>(info.status_code[idx]);
      if (idx != 0) {
        ss << " ";
      }
    }    
    write.String(ss.str().c_str());
  }
  
  if (key_mask.find(kKeyLidarDiagStatus) != key_mask.end()) {
    write.Key("lidar_diag_status");
    write.Uint(info.lidar_diag_status);
  }
  
  if (key_mask.find(kKeyLidarFlashStatus) != key_mask.end()) {
    write.Key("lidar_flash_status");
    write.Uint(info.lidar_flash_status);
  }
  
  if (key_mask.find(kKeyFwType) != key_mask.end()) {
    write.Key("FW_TYPE");
    write.Uint(info.fw_type);
  }
  
  if (key_mask.find(kKeyHmsCode) != key_mask.end()) {
    write.Key("hms_code");
    write.StartArray();
    write.Uint(info.hms_code[0]);
    write.Uint(info.hms_code[1]);
    write.Uint(info.hms_code[2]);
    write.Uint(info.hms_code[3]);
    write.Uint(info.hms_code[4]);
    write.Uint(info.hms_code[5]);
    write.Uint(info.hms_code[6]);
    write.Uint(info.hms_code[7]);
    write.EndArray();
  }
  
  if (key_mask.find(kKeyRoiMode) != key_mask.end()) {
    write.Key("ROI_Mode");
    write.Uint(info.ROI_Mode);
  } 

  write.EndObject();

  lidar_info = buf.GetString();
  // LOG_INFO("###################################lidar_info_to_json:{}", lidar_info.c_str());
}

} // namespace livox
} // namespace direct





