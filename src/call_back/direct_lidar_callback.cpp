#include "direct_lidar_callback.h"

namespace livox_ros {

void DirectLidarCallback::DirectLidarInfoCb(const uint32_t handle, DirectLidarStateInfo* info, void* client_data) {
  if (info == NULL) {
    printf("Direct lidar info callback failed, the info is null.\n");
    return;
  }

  if (client_data == NULL) {
    printf("Direct lidar info callback failed, client data is nullptr.\n");
    return;
  }
  
  // printf("Direct lidar info callback, the sn:%s, lidar_id:%u, product_info:%s, version_app:%s, lidar_ip:%s.\n",
  //     info->sn, info->lidar_id, info->product_info, info->version_app, info->lidar_ip);
  // printf("Direct lidar info callback, the host_ip:%s, push_port:%u, data_ip:%s, data_port:%u\n", info->host_push_msg_ip,
  //     info->host_push_msg_port, info->host_point_data_ip, info->host_point_data_port);

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  lds_lidar->SetDirectLidarExtrinsicParamsByDeviceInfo(handle, info->install_attitude);
  lds_lidar->ProcessDirectConfigByDeviceInfo(handle, info);
}

void DirectLidarCallback::DirectLidarCfgUpdateCb(const uint32_t handle, DirectLidarCmdResInfo* response, void* client_data) {
  if (response == nullptr) {
    printf("SetDirectLidarCallback response is nullptr.\n");
    return;
  }

  if (client_data == nullptr) {
    printf("SetDirectLidarCallback client_data is nullptr.\n");
    return;
  }
  
  if (response->res.ret_code == 0) {
    printf("Set direct lidar succ, the sn:%s, handle:%d.\n", response->sn, handle);
    LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
    if (lds_lidar != nullptr) {
      lds_lidar->DirectLidarStartSamp(handle);
    }
  }
}
} // namespace livox_ros
