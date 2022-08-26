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

#include "pa_command_handler.h"
#include "livox_lidar_def.h"

#include "base/command_callback.h"

#include "base/logging.h"
#include "comm/protocol.h"
#include "comm/generate_seq.h"
#include "build_request.h"
#include "general_command_handler.h"

namespace livox {

namespace lidar {

PaCommandHandler::PaCommandHandler(DeviceManager* device_manager)
    : CommandHandler(device_manager), comm_port_(new CommPort) {
}


bool PaCommandHandler::Init(bool is_view) {

  return true;
}

bool PaCommandHandler::Init(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
    std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr) {

  return true;
}

void PaCommandHandler::Handle(const uint32_t handle, uint16_t lidar_port, const Command& command) {

}

void PaCommandHandler::OnCommandAck(uint32_t handle, const Command &command) {

}

void PaCommandHandler::OnCommandCmd(uint32_t handle, const Command &command) {
}

void PaCommandHandler::UpdateLidarCfg(const ViewLidarIpInfo& view_lidar_info) {

}

void PaCommandHandler::UpdateLidarCfg(const uint32_t handle, const uint16_t lidar_cmd_port) {

}

void PaCommandHandler::SetViewLidar(const ViewLidarIpInfo& view_lidar_info) {

}

void PaCommandHandler::SetCustomLidar(const uint32_t handle, const uint16_t lidar_cmd_port, const LivoxLidarCfg& lidar_cfg) {

}

void PaCommandHandler::SetGeneralLidar(const uint32_t handle, const uint16_t lidar_cmd_port) {

}

void PaCommandHandler::UpdateLidarCallback(livox_status status, uint32_t handle,
    LivoxLidarAsyncControlResponse *response, void *client_data) {

}

void PaCommandHandler::AddDevice(const uint32_t handle) {

}

bool PaCommandHandler::IsStatusException(const Command &command) {

  return true;
}

livox_status PaCommandHandler::SendCommand(const Command &command, const uint16_t lidar_cmd_port) {

  return kLivoxLidarStatusSuccess;
}

livox_status PaCommandHandler::SendCommand(const Command &command) {

  return kLivoxLidarStatusSuccess;
}


bool PaCommandHandler::GetHostInfo(const uint32_t handle, std::string& host_ip, uint16_t& cmd_port) {
  return true;
}

}  // namespace livox
} // namespace lidar

