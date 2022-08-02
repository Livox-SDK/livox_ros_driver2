//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
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

#ifndef COMMAND_HANDLER_H_
#define COMMAND_HANDLER_H_

#include <memory>
#include <map>
#include <mutex>

#include "comm/define.h"
#include "base/command_callback.h"
#include "base/io_thread.h"
#include <memory>

#include "livox_def_direct.h"


#include "comm/protocol.h"
#include "comm/define.h"
#include "build_request.h"


#include "command_handler_impl.h"

namespace livox {
namespace direct {

class CommandHandlerImpl;

class CommandHandler : public noncopyable {
 public:
  CommandHandler() {}
  ~CommandHandler() {};

  bool Init(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr,
    std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr,
    const uint8_t lidars_num);

  bool Start();

  void Destory();

  void LidarStateInfoCallback(const uint32_t handle, DirectLidarStateInfo* info);
  void DirectLidarCfgUpdateCallback(const uint32_t handle, DirectLidarCmdResInfo* res);

  void SetLidarInfoCallback(LidarInfoCallback cb, void* client_data) {
    lidar_info_callback_ = cb;
    lidar_info_client_data_ = client_data;
  }

  void SetLidarCfgUpdateCallback(LidarCommandCallback cb, void* client_data) {
    lidar_cfg_update_callback_ = cb;
    lidar_cfg_update_client_data_ = client_data;
  }

  livox_direct_status UpdateDirectLidarCfg(std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr);

  livox_direct_status SendCommand(uint32_t handle,
                                  uint16_t command_id,
                                  uint8_t *data,
                                  uint16_t length,
                                  const std::shared_ptr<CommandCallback> &cb);

 private:
  void ParseLidarStateInfo(const CommPacket& packet, DirectLidarStateInfo& info);
  bool GetLidarInfo(const uint32_t handle, uint8_t& lidar_id, std::string& sn, std::string& lidar_ip);
 private:
  std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr_;
  std::unique_ptr<CommandHandlerImpl> impl_;

  LidarCommandCallback lidar_cfg_update_callback_;
  void* lidar_cfg_update_client_data_ = nullptr;

  LidarInfoCallback lidar_info_callback_;
  void* lidar_info_client_data_ = nullptr;

  std::mutex mutex_;
  std::map<uint32_t, DirectLidarStateInfo> handle_lidar_info_;

};

CommandHandler &command_handler();

}  // namespace livox
} // namespace direct

#endif  // COMMAND_HANDLER_H_