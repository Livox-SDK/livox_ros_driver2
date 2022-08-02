#include "parse_input_params.h"

#include "base/logging.h"

namespace livox { 

namespace direct {

ParseInputParams::ParseInputParams(DirectLidarHostCfg* direct_host_cfg_ptr, DirectLidarCfg* direct_lidar_cfg_ptr,
    const uint8_t lidars_num) : direct_host_cfg_ptr_(direct_host_cfg_ptr),
    direct_lidar_cfg_ptr_(direct_lidar_cfg_ptr), lidars_num_(lidars_num) {}

bool ParseInputParams::Parse(std::shared_ptr<DirectHostIpInfo>& direct_host_ipinfo_ptr,
    std::shared_ptr<std::vector<DirectLidarInfo>>& direct_lidars_info_ptr) {
  if (direct_host_cfg_ptr_ == nullptr && direct_lidar_cfg_ptr_ == nullptr) {
    LOG_ERROR("Parse input params failed, pointers to input parameters are null");
    return false;
  }
  if (lidars_num_ == 0 || lidars_num_ >= kMaxLidarCount) {
    LOG_ERROR("Parse input params failed, lidar_num is zero or exceeds the maximum 32");
    return false;
  }

  if (direct_host_cfg_ptr_) {
    return ParseHostIpInfo(direct_host_cfg_ptr_, direct_host_ipinfo_ptr);
  }

  return ParseLidarsInfo(direct_lidars_info_ptr);
}

bool ParseInputParams::ParseHostIpInfo(const DirectLidarHostCfg* host_cfg_ptr,
    std::shared_ptr<DirectHostIpInfo>& direct_host_info_ptr) {
  if (host_cfg_ptr == nullptr) {
    return false;
  }

  if (strlen(host_cfg_ptr->host_push_cmd_ip) == 0 ||
      strlen(host_cfg_ptr->host_point_data_ip) == 0 ||
      strlen(host_cfg_ptr->host_imu_data_ip) == 0) {
    LOG_ERROR("Parse host ip info failed, host_push_cmd_ip and host_point_data_ip and \
        host_imu_data_ip contain empty characters");
    return false;
  }

  direct_host_info_ptr.reset(new DirectHostIpInfo());

  direct_host_info_ptr->host_push_msg_ip = host_cfg_ptr->host_push_cmd_ip;
  direct_host_info_ptr->host_point_data_ip = host_cfg_ptr->host_point_data_ip;
  direct_host_info_ptr->host_imu_data_ip = host_cfg_ptr->host_imu_data_ip;

  direct_host_info_ptr->host_cmd_port = kHostCmdPort;
  if (host_cfg_ptr->host_cmd_port != 0) {
    direct_host_info_ptr->host_cmd_port = host_cfg_ptr->host_cmd_port;
  }

  direct_host_info_ptr->host_push_cmd_port = kHostPushCmdPort;
  if (host_cfg_ptr->host_push_cmd_port != 0) {
    direct_host_info_ptr->host_push_cmd_port = host_cfg_ptr->host_push_cmd_port;
  }

  direct_host_info_ptr->host_point_data_port = kHostPointDataPort;
  if (host_cfg_ptr->host_point_data_port != 0) {
    direct_host_info_ptr->host_point_data_port = host_cfg_ptr->host_point_data_port;
  }

  direct_host_info_ptr->host_imu_data_port = kHostImuDataPort;
  if (host_cfg_ptr->host_imu_data_port != 0) {
    direct_host_info_ptr->host_imu_data_port = host_cfg_ptr->host_imu_data_port;
  }

  direct_host_info_ptr->host_log_port = kHostLogPort;
  if (host_cfg_ptr->host_log_port != 0) {
    direct_host_info_ptr->host_log_port = host_cfg_ptr->host_log_port;
  }

  LOG_INFO("Parse the input host params, host_push_msg_ip:{}, host_point_data_ip:{}, host_imu_data_ip:{}",
      direct_host_info_ptr->host_push_msg_ip.c_str(), direct_host_info_ptr->host_point_data_ip.c_str(),
      direct_host_info_ptr->host_imu_data_ip.c_str());

  LOG_INFO("Parse the input host params, host_cmd_port:{}, host_push_cmd_port:{}, "
      "host_point_data_port:{}, host_imu_data_port:{}, host_log_port:{}",
      direct_host_info_ptr->host_cmd_port, direct_host_info_ptr->host_push_cmd_port,
      direct_host_info_ptr->host_point_data_port, direct_host_info_ptr->host_imu_data_port,
      direct_host_info_ptr->host_log_port);
  return true;
}

bool ParseInputParams::ParseLidarsInfo(std::shared_ptr<std::vector<DirectLidarInfo>>& direct_lidars_info_ptr) {
  if (direct_lidar_cfg_ptr_ == nullptr) {
    return false;
  }

  direct_lidars_info_ptr.reset(new std::vector<DirectLidarInfo>());

  for (uint8_t i = 0; i < lidars_num_; ++i) {
    const DirectLidarCfg& direct_lidar_cfg = direct_lidar_cfg_ptr_[i];
    std::string sn = direct_lidar_cfg.sn;
    if (sn.empty()) {
      LOG_ERROR("Parse lidars info failed, the sn is empty.");
      return false;
    }

    // (*direct_lidars_info_ptr)[sn] = DirectLidarInfo();
    // DirectLidarInfo& direct_lidar_info = (*direct_lidars_info_ptr)[sn];

    DirectLidarInfo direct_lidar_info;
    direct_lidar_info.sn = sn;
    direct_lidar_info.lidar_id = direct_lidar_cfg.lidar_id;
    direct_lidar_info.lidar_ipmode = direct_lidar_cfg.lidar_ipmode;
    LOG_INFO("Parse Input Params, sn:{}, lidar_id:{}, lidar_ipmode:{}", direct_lidar_info.sn.c_str(),
        direct_lidar_info.lidar_id, direct_lidar_info.lidar_ipmode);
    
    if (!ParseLidarIpInfo(direct_lidar_cfg.lidar_ipinfo_cfg, direct_lidar_info.lidar_ipinfo_ptr)) {
      return false;
    }

    if (!ParseHostIpInfo(&direct_lidar_cfg.host_cfg, direct_lidar_info.host_ipinfo_ptr)) {
      return false;
    }

    direct_lidar_info.sample_mode = direct_lidar_cfg.sample_mode;


    direct_lidar_info.pattern_mode = direct_lidar_cfg.pattern_mode;
    direct_lidar_info.pcl_data_type = direct_lidar_cfg.pcl_data_type;
    direct_lidar_info.imu_data_en = direct_lidar_cfg.imu_data_en;
    direct_lidar_info.work_mode = direct_lidar_cfg.work_mode;

    // LOG_INFO("Parse Input Params, sample_mode{}, pattern_mode:{}, pcl_data_type:{}, "
    //     "imu_data_en:{}, work_mode:{}", direct_lidar_info.sample_mode, direct_lidar_info.pattern_mode,
    //     direct_lidar_info.pattern_mode, direct_lidar_info.imu_data_en, direct_lidar_info.work_mode);

    memcpy(&direct_lidar_info.install_attitude, &direct_lidar_cfg.install_attitude, sizeof(InstallAttitude));
    // LOG_INFO("Parse Input Params, the install attitude, roll_deg:{}, pitch_deg:{}, yaw_deg:{}, x:{}, y:{}, z:{}",
    //     direct_lidar_info.install_attitude.roll_deg, direct_lidar_info.install_attitude.pitch_deg,
    //     direct_lidar_info.install_attitude.yaw_deg, direct_lidar_info.install_attitude.x,
    //     direct_lidar_info.install_attitude.y, direct_lidar_info.install_attitude.z);

    direct_lidars_info_ptr->push_back(std::move(direct_lidar_info));
  }
  return true;
}

bool ParseInputParams::ParseLidarIpInfo(const DirectLidarIpCfg& lidar_ipinfo_cfg,
    std::shared_ptr<DirectLidarIpInfo>& direct_lidar_ipinfo_ptr) {
  if (strlen(lidar_ipinfo_cfg.lidar_ipaddr) == 0) {
    LOG_ERROR("Parse lidar ip info failed, the lidar_ipaddr is empty.");
    return false;
  }
  if (strlen(lidar_ipinfo_cfg.lidar_subnet_mask) == 0) {
    LOG_ERROR("Parse lidar ip info failed, the lidar_subnet_mask is empty.");
    return false;
  }
  if (strlen(lidar_ipinfo_cfg.lidar_gateway) == 0) {
    LOG_ERROR("Parse lidar ip info failed, the lidar_gateway is empty.");
    return false;
  }

  direct_lidar_ipinfo_ptr.reset(new DirectLidarIpInfo());
  direct_lidar_ipinfo_ptr->lidar_ipaddr = lidar_ipinfo_cfg.lidar_ipaddr;
  direct_lidar_ipinfo_ptr->lidar_subnet_mask = lidar_ipinfo_cfg.lidar_subnet_mask;
  direct_lidar_ipinfo_ptr->lidar_gateway = lidar_ipinfo_cfg.lidar_gateway;

  direct_lidar_ipinfo_ptr->lidar_cmd_port = kLidarCmdPort;
  direct_lidar_ipinfo_ptr->lidar_push_cmd_port = kLidarPushCmdPort;
  direct_lidar_ipinfo_ptr->lidar_point_data_port = kLidarPointDataPort;
  direct_lidar_ipinfo_ptr->lidar_imu_data_port = kLidarImuDataPort;
  direct_lidar_ipinfo_ptr->lidar_log_port = kLidarLogPort;

  LOG_INFO("Parse Input Params, the lidar ip info, lidar_ipaddr:{}, lidar_subnet_mask:{}, lidar_gateway{}",
      direct_lidar_ipinfo_ptr->lidar_ipaddr.c_str(), direct_lidar_ipinfo_ptr->lidar_subnet_mask.c_str(),
      direct_lidar_ipinfo_ptr->lidar_gateway.c_str());
  LOG_INFO("Parse Input Params, the lidar port info, lidar_cmd_port:{}, lidar_push_cmd_port:{},"
      "lidar_point_data_port:{}, lidar_imu_data_port:{}, lidar_log_port:{}",
      direct_lidar_ipinfo_ptr->lidar_cmd_port, direct_lidar_ipinfo_ptr->lidar_push_cmd_port,
      direct_lidar_ipinfo_ptr->lidar_point_data_port, direct_lidar_ipinfo_ptr->lidar_imu_data_port,
      direct_lidar_ipinfo_ptr->lidar_log_port);

  return true;
}

} // namespace livox

} // namespace direct
