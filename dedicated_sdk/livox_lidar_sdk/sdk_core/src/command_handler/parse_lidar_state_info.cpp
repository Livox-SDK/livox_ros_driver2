
#include "parse_lidar_state_info.h"
#include "base/logging.h"

namespace livox {

namespace lidar {

bool ParseLidarStateInfo::Parse(const CommPacket& packet, std::string& info_str) {
  DirectLidarStateInfo info;
  if (!ParseStateInfo(packet, info)) {
    return false;
  }

  LivoxLidarStateInfoToJson(info, info_str);
  return true;
}

bool ParseLidarStateInfo::ParseStateInfo(const CommPacket& packet, DirectLidarStateInfo& info) {  
  uint16_t off = 0;
  uint16_t key_num = 0;
  memcpy(&key_num, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t) * 2;  

  for (uint16_t i = 0; i < key_num; ++i) {
    if (off + sizeof(LivoxLidarKeyValueParam) > packet.data_len) {
      return false;
    }

    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&packet.data[off];
    off += sizeof(uint16_t);

    uint16_t val_len = 0;
    memcpy(&val_len, &packet.data[off], sizeof(uint16_t));
    off += sizeof(uint16_t);
  
    switch (kv->key) {
      case static_cast<uint16_t>(kKeyPclDataType) :
        memcpy(&info.pcl_data_type, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyPatternMode) :
        memcpy(&info.pattern_mode, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyLidarIPCfg) :
        ParseLidarIpAddr(packet, off, info);
        break;
      case static_cast<uint16_t>(kKeyStateInfoHostIPCfg) :
        ParseStateInfoHostIPCfg(packet, off, info);
        break;
      case static_cast<uint16_t>(kKeyLidarPointDataHostIPCfg) :
        ParsePointCloudHostIpCfg(packet, off, info);
        break;
      case static_cast<uint16_t>(kKeyLidarImuHostIPCfg) :
        ParseImuDataHostIpCfg(packet, off, info);
        break;
      case static_cast<uint16_t>(kKeyInstallAttitude) :
        memcpy(&info.install_attitude, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyRoiCfg0) :
        memcpy(&info.roi_cfg0, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyRoiCfg1) :
        memcpy(&info.roi_cfg1, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyRoiEn) :
        memcpy(&info.roi_en, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyWorkMode) :
        memcpy(&info.work_mode, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyImuDataEn) :
        memcpy(&info.imu_data_en, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeySn) :
        memcpy(info.sn, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyProductInfo) :
        memcpy(info.product_info, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyVersionApp) :
        memcpy(info.version_app, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyVersionLoader) :
        memcpy(info.version_load, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyVersionHardware):
        memcpy(info.version_hardware, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyMac) :
        memcpy(info.mac, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyCurWorkState) :
        memcpy(&info.cur_work_state, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyCoreTemp) :
        memcpy(&info.core_temp, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyPowerUpCnt) :
        memcpy(&info.powerup_cnt, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyLocalTimeNow) :
        memcpy(&info.local_time_now, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyLastSyncTime) :
        memcpy(&info.last_sync_time, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyTimeOffset) :
        memcpy(&info.time_offset, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyTimeSyncType) :
        memcpy(&info.time_sync_type, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyFwType) :
        memcpy(&info.fw_type, &packet.data[off], val_len);
        off += val_len;
        break;  
      default :
        off += val_len;
    }
  }

  // printf("Lidar state info, pcl_data_type:%d, pattern_mode:%d, lidar_ip:%s, lidar_submask:%s, lidar_gatway:%s.\n",
  //     info.pcl_data_type, info.pattern_mode, info.livox_lidar_ip_info.ip_addr, info.livox_lidar_ip_info.net_mask, 
  //     info.livox_lidar_ip_info.gw_addr);
  
  // printf("Lidar state info, host_ip_addr:%s, host_state_info_port:%u, lidar_state_info_port:%u.\n",
  //     info.host_state_info.host_ip_addr, info.host_state_info.host_state_info_port, info.host_state_info.lidar_state_info_port);

  // printf("Lidar state info, host_ip_addr:%s, host_point_data_port:%u, lidar_point_data_port:%u.\n",
  //     info.host_point_ip_info.host_ip_addr, info.host_point_ip_info.host_point_data_port,
  //     info.host_point_ip_info.lidar_point_data_port);
    
  // printf("Lidar state info, host_ip_addr:%s, host_imu_data_port:%u, lidar_imu_data_port:%u.\n",
  //     info.host_imu_data_ip_info.host_ip_addr, info.host_point_ip_info.host_point_data_port,
  //     info.host_imu_data_ip_info.lidar_imu_data_port);

  // printf("Lidar state info, roll:%f, pitch:%f, yaw:%f, x:%d, y:%d,z:%d.\n",
  //        info.install_attitude.roll_deg, info.install_attitude.pitch_deg,
  //        info.install_attitude.yaw_deg, info.install_attitude.x, info.install_attitude.y, info.install_attitude.z);
  
  // printf("Lidar state info, roi cfg0, yaw_start:%d, yaw_stop:%d, pitch_start:%d, pitch_stop:%d.\n",
  //     info.roi_cfg0.yaw_start, info.roi_cfg0.yaw_stop, info.roi_cfg0.pitch_start, info.roi_cfg0.pitch_stop);

  // printf("Lidar state info, roi cfg1, yaw_start:%d, yaw_stop:%d, pitch_start:%d, pitch_stop:%d.\n",
  //     info.roi_cfg1.yaw_start, info.roi_cfg1.yaw_stop, info.roi_cfg1.pitch_start, info.roi_cfg1.pitch_stop);

  // printf("Lidar state info, roi_en:%u, work_mode:%u, imu_data_en:%u, sn:%s, product_info:%s.\n",
  //     info.roi_en, info.work_mode, info.imu_data_en, info.sn, info.product_info);

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

void ParseLidarStateInfo::ParseLidarIpAddr(const CommPacket& packet, uint16_t& off, DirectLidarStateInfo& info) {
  uint8_t lidar_ip[4];
  memcpy(lidar_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_ip_str = std::to_string(lidar_ip[0]) + "." + std::to_string(lidar_ip[1]) + "." + 
      std::to_string(lidar_ip[2]) + "." + std::to_string(lidar_ip[3]);
  strcpy(info.livox_lidar_ip_info.ip_addr, lidar_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  uint8_t lidar_submask[4];
  memcpy(lidar_submask, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_submask_str = std::to_string(lidar_submask[0]) + "." + std::to_string(lidar_submask[1]) + 
      "." + std::to_string(lidar_submask[2]) + "." + std::to_string(lidar_submask[3]);
  strcpy(info.livox_lidar_ip_info.net_mask, lidar_submask_str.c_str());
  off += sizeof(uint8_t) * 4;
  
  uint8_t lidar_gateway[4];
  memcpy(lidar_gateway, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_gateway_str = std::to_string(lidar_gateway[0]) + "." + std::to_string(lidar_gateway[1]) +
      "." + std::to_string(lidar_gateway[2]) + "." + std::to_string(lidar_gateway[3]);
  strcpy(info.livox_lidar_ip_info.gw_addr, lidar_gateway_str.c_str());
  off += sizeof(uint8_t) * 4;
}

void ParseLidarStateInfo::ParseStateInfoHostIPCfg(const CommPacket& packet, uint16_t& off, DirectLidarStateInfo& info) {
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
  off += sizeof(uint16_t);
}

void ParseLidarStateInfo::ParsePointCloudHostIpCfg(const CommPacket& packet, uint16_t& off, DirectLidarStateInfo& info) {
  uint8_t host_point_cloud_ip[4];
  memcpy(host_point_cloud_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_point_cloud_ip_str = std::to_string(host_point_cloud_ip[0]) + "." + 
      std::to_string(host_point_cloud_ip[1]) + "." + std::to_string(host_point_cloud_ip[2]) + "." +
      std::to_string(host_point_cloud_ip[3]);
  
  strcpy(info.host_point_ip_info.host_ip_addr, host_point_cloud_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.host_point_ip_info.host_point_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);

  memcpy(&info.host_point_ip_info.lidar_point_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);
}

void ParseLidarStateInfo::ParseImuDataHostIpCfg(const CommPacket& packet, uint16_t& off, DirectLidarStateInfo& info) {
  uint8_t host_imu_data_ip[4];
  memcpy(host_imu_data_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_imu_data_ip_str = std::to_string(host_imu_data_ip[0]) + "." + 
      std::to_string(host_imu_data_ip[1]) + "." + std::to_string(host_imu_data_ip[2]) + "." +
      std::to_string(host_imu_data_ip[3]);
  
  strcpy(info.host_imu_data_ip_info.host_ip_addr, host_imu_data_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.host_imu_data_ip_info.host_imu_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);

  memcpy(&info.host_imu_data_ip_info.lidar_imu_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);
}

void ParseLidarStateInfo::LivoxLidarStateInfoToJson(const DirectLidarStateInfo& info, std::string& lidar_info) {
  rapidjson::StringBuffer buf;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> write(buf);
  write.StartObject();

  write.Key("dev_type");
  write.String("MID360");

  write.Key("pcl_data_type");
  write.Uint(info.pcl_data_type);

  write.Key("pattern_mode");
  write.Uint(info.pattern_mode);

  write.Key("lidar_ipinfo");
  write.StartObject();
  write.Key("lidar_ipaddr");
  write.String(info.livox_lidar_ip_info.ip_addr);

  write.Key("lidar_subnet_mask");
  write.String(info.livox_lidar_ip_info.net_mask);

  write.Key("lidar_gateway");
  write.String(info.livox_lidar_ip_info.gw_addr);
  write.EndObject();

  write.Key("state_info_host_ipinfo");
  write.StartObject();
  write.Key("host_ip_addr");
  write.String(info.host_state_info.host_ip_addr);
  write.Key("host_state_info_port");
  write.Uint(info.host_state_info.host_state_info_port);
  write.Key("lidar_state_info_port");
  write.Uint(info.host_state_info.lidar_state_info_port);
  write.EndObject();

  write.Key("point_cloud_host_ipinfo");
  write.StartObject();
  write.Key("host_ip_addr");
  write.String(info.host_point_ip_info.host_ip_addr);
  write.Key("host_point_cloud_port");
  write.Uint(info.host_point_ip_info.host_point_data_port);
  write.Key("lidar_point_cloud_port");
  write.Uint(info.host_point_ip_info.lidar_point_data_port);
  write.EndObject();

  write.Key("imu_host_ipinfo");
  write.StartObject();
  write.Key("host_ip_addr");
  write.String(info.host_imu_data_ip_info.host_ip_addr);
  write.Key("host_imu_data_port");
  write.Uint(info.host_imu_data_ip_info.host_imu_data_port);
  write.Key("lidar_imu_data_port");
  write.Uint(info.host_imu_data_ip_info.lidar_imu_data_port);
  write.EndObject();

  write.Key("install_attitude");
  write.StartObject();
  write.Key("roll_deg");
  write.Double(info.install_attitude.roll_deg);
  write.Key("pitch_deg");
  write.Double(info.install_attitude.pitch_deg);
  write.Key("yaw_deg");
  write.Double(info.install_attitude.yaw_deg);
  write.Key("x");
  write.Uint(info.install_attitude.x);
  write.Key("y");
  write.Uint(info.install_attitude.y);
  write.Key("z");
  write.Uint(info.install_attitude.z);
  write.EndObject();

  write.Key("roi_cfg0");
  write.StartObject();
  write.Key("yaw_start");
  write.Int(info.roi_cfg0.yaw_start);
  write.Key("yaw_stop");
  write.Int(info.roi_cfg0.yaw_stop);
  write.Key("pitch_start");
  write.Int(info.roi_cfg0.pitch_start);
  write.Key("pitch_stop");
  write.Int(info.roi_cfg0.pitch_stop);
  write.EndObject();

  write.Key("roi_cfg1");
  write.StartObject();
  write.Key("yaw_start");
  write.Int(info.roi_cfg0.yaw_start);
  write.Key("yaw_stop");
  write.Int(info.roi_cfg0.yaw_stop);
  write.Key("pitch_start");
  write.Int(info.roi_cfg0.pitch_start);
  write.Key("pitch_stop");
  write.Int(info.roi_cfg0.pitch_stop);
  write.EndObject();

  write.Key("roi_en");
  write.Uint(info.roi_en);

  write.Key("work_mode");
  write.Uint(info.work_mode);

  write.Key("imu_data_en");
  write.Uint(info.imu_data_en);

  write.Key("sn");
  write.String(info.sn);

  write.Key("product_info");
  write.String(info.product_info);

  write.Key("version_app");
  write.StartArray();
  write.Uint(info.version_app[0]);
  write.Uint(info.version_app[1]);
  write.Uint(info.version_app[2]);
  write.Uint(info.version_app[3]);
  write.EndArray();

  write.Key("version_loader");
  write.StartArray();
  write.Uint(info.version_load[0]);
  write.Uint(info.version_load[1]);
  write.Uint(info.version_load[2]);
  write.Uint(info.version_load[3]);
  write.EndArray();

  write.Key("version_hardware");
  write.StartArray();
  write.Uint(info.version_hardware[0]);
  write.Uint(info.version_hardware[1]);
  write.Uint(info.version_hardware[2]);
  write.Uint(info.version_hardware[3]);
  write.EndArray();

  write.Key("mac");
  write.StartArray();
  write.Uint(info.mac[0]);
  write.Uint(info.mac[1]);
  write.Uint(info.mac[2]);
  write.Uint(info.mac[3]);
  write.Uint(info.mac[4]);
  write.Uint(info.mac[5]);
  write.EndArray();

  write.Key("cur_work_state");
  write.Uint(info.cur_work_state);

  write.Key("core_temp");
  write.Int(info.core_temp);

  write.Key("powerup_cnt");
  write.Uint(info.powerup_cnt);

  write.Key("local_time_now");
  write.Uint64(info.local_time_now);

  write.Key("last_sync_time");
  write.Uint64(info.last_sync_time);
  
  write.Key("time_offset");
  write.Int64(info.time_offset);

  write.Key("time_sync_type");
  write.Uint(info.time_sync_type);

  write.Key("fw_type");
  write.Uint(info.fw_type);

  write.EndObject();

  lidar_info = buf.GetString();
  // LOG_INFO("###################################lidar_info_to_json:{}", lidar_info.c_str());
}

} // namespace livox
} // namespace direct





