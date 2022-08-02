
#include "parse_lidar_state_info.h"
#include "base/logging.h"

namespace livox {

namespace direct {

bool ParseLidarStateInfo::Parse(const CommPacket& packet, DirectLidarStateInfo& info) {  
  uint16_t off = 0;
  uint16_t list_len = 0;
  memcpy(&list_len, &packet.data[off], sizeof(uint16_t));
  off = off + sizeof(DirectLidarStateInfoHead);  

  for (uint16_t i = 0; i < list_len; ++i) {
    if (off + sizeof(DirectKeyValueParam) > packet.data_len) {
      return false;
    }

    DirectKeyValueParam* kv = (DirectKeyValueParam*)&packet.data[off];
    off += sizeof(uint16_t);

    uint16_t val_len = 0;
    memcpy(&val_len, &packet.data[off], sizeof(uint16_t));
    off += sizeof(uint16_t);
    switch (kv->key) {
      case static_cast<uint16_t>(kKeySn) :
        memcpy(info.sn, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyProductInfo) :
        memcpy(info.product_info, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyVersionApp) :
        ParseLidarVersionApp(packet, off, val_len, info);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyVersionLoader) :
        memcpy(info.version_loader, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyVersionHardWare):
        memcpy(info.version_hardware, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyLidarMac) :
        ParseLidarMac(packet, off, val_len, info);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyLidarID) :
        memcpy(&info.lidar_id, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyLidarIpMode) :
        memcpy(&info.lidar_ipmode, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyLidarIpAddr) :
        ParseLidarIpAddr(packet, off, info);
        break;
      case static_cast<uint16_t>(kKyeHostIpAddr) :
        ParseHostIpAddr(packet, off, info);
        break;
      case static_cast<uint16_t>(kKeySampleMode) :
        memcpy(&info.sample_mode, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyPatternMode) :
        memcpy(&info.pattern_mode, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyPclDataType) :
        memcpy(&info.pcl_data_type, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyImuDataEn) :
        memcpy(&info.imu_data_en, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyInstallAttitude) :
        memcpy(&info.install_attitude, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyWorkMode) :
        memcpy(&info.work_mode, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyWorkState) :
        memcpy(&info.work_state, &packet.data[off], val_len);
        off += val_len;
        break;
      case static_cast<uint16_t>(kKeyCoreTemp) :
        memcpy(&info.core_temp, &packet.data[off], val_len);
        off += val_len;
        break;
      default :
        off += val_len;
        //LOG_INFO("Parse lidar state info, unknown key:{}, val_len:{}", kv->key, val_len);
    }
  }

  // printf("Lidar state info sn:%s, product_info:%s, version_app:%s, version_loader:%u,%u,%u,%u.\n"
  //     "version_hardware:%u,%u,%u,%u,", info.sn, info.product_info, info.version_app, info.version_loader[0],
  //     info.version_loader[1], info.version_loader[2], info.version_loader[3], info.version_hardware[0],
  //     info.version_hardware[1], info.version_hardware[2], info.version_hardware[3]);
  
  // printf("Lidar state info lidar_mac:%s, lidar_id:%u, lidar_ipmode:%u, lidar_ip:%s, lidar_submask:%s, lidar_gatway:%s, host_push_ip:%s.\n",
  //     info.lidar_mac, info.lidar_id, info.lidar_ipmode, info.lidar_ip, info.lidar_submask, info.lidar_gateway, info.host_push_msg_ip);
  
  // printf("Lidar state info host_push_msg_port:%u, host_point_data_ip:%s, host_point_data_port:%u, host_imu_data_ip:%s, host_imu_data_port:%u.\n",
  //     info.host_push_msg_port, info.host_point_data_ip, info.host_point_data_port, info.host_imu_data_ip, info.host_imu_data_port);
  
  // printf("Lidar state info sample_mode:%d, pattern_mode:%d, pcl_data_type:%d, imu_data_en:%d.\n",
  //        info.sample_mode, info.pattern_mode, info.pcl_data_type, info.imu_data_en);
  // printf("Lidar state info roll:%f, pitch:%f, yaw:%f, x:%d, y:%d,z:%d.\n",
  //        info.install_attitude.roll_deg, info.install_attitude.pitch_deg,
  //        info.install_attitude.yaw_deg, info.install_attitude.x, info.install_attitude.y, info.install_attitude.z);

  // printf("Lidar state info work_mode:%u, core_temp:%d.\n", info.work_mode, info.core_temp);
  return true;
}

void ParseLidarStateInfo::ParseLidarVersionApp(const CommPacket& packet, uint16_t& off, uint16_t val_len, DirectLidarStateInfo& info) {
  uint8_t version_app[4];
  memcpy(version_app, &packet.data[off], val_len);
  std::string version_app_str = std::to_string(version_app[0]) + "." + std::to_string(version_app[1]) + "." +
      std::to_string(version_app[2]) + "." + std::to_string(version_app[3]);
  strcpy(info.version_app, version_app_str.c_str());
}

void ParseLidarStateInfo::ParseLidarMac(const CommPacket& packet, uint16_t& off, uint16_t val_len,
     DirectLidarStateInfo& info) {  
  uint8_t lidar_mac[6];
  memcpy(lidar_mac, &packet.data[off], val_len);
  std::string lidar_mac_str;
  for (uint8_t i = 0; i < 5; ++i) {
    lidar_mac_str += std::to_string(lidar_mac[i]) + ":";
  }
  lidar_mac_str += std::to_string(lidar_mac[5]);
  strcpy(info.lidar_mac, lidar_mac_str.c_str());
}

void ParseLidarStateInfo::ParseLidarIpAddr(const CommPacket& packet, uint16_t& off, DirectLidarStateInfo& info) {
  uint8_t lidar_ip[4];
  memcpy(lidar_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_ip_str = std::to_string(lidar_ip[0]) + "." + std::to_string(lidar_ip[1]) + "." + 
      std::to_string(lidar_ip[2]) + "." + std::to_string(lidar_ip[3]);
  strcpy(info.lidar_ip, lidar_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  uint8_t lidar_submask[4];
  memcpy(lidar_submask, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_submask_str = std::to_string(lidar_submask[0]) + "." + std::to_string(lidar_submask[1]) + 
      "." + std::to_string(lidar_submask[2]) + "." + std::to_string(lidar_submask[3]);
  strcpy(info.lidar_submask, lidar_submask_str.c_str());
  off += sizeof(uint8_t) * 4;
  
  uint8_t lidar_gateway[4];
  memcpy(lidar_gateway, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_gateway_str = std::to_string(lidar_gateway[0]) + "." + std::to_string(lidar_gateway[1]) +
      "." + std::to_string(lidar_gateway[2]) + "." + std::to_string(lidar_gateway[3]);
  strcpy(info.lidar_gateway, lidar_gateway_str.c_str());
  off += sizeof(uint8_t) * 4;
}

void ParseLidarStateInfo::ParseHostIpAddr(const CommPacket& packet, uint16_t& off, DirectLidarStateInfo& info) {
  uint8_t host_push_msg_ip[4];
  memcpy(host_push_msg_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_push_msg_ip_str = std::to_string(host_push_msg_ip[0]) + "." + 
      std::to_string(host_push_msg_ip[1]) + "." + std::to_string(host_push_msg_ip[2]) + "." +
      std::to_string(host_push_msg_ip[3]);
  strcpy(info.host_push_msg_ip, host_push_msg_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.host_push_msg_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);


  uint8_t host_point_data_ip[4];
  memcpy(host_point_data_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_point_data_ip_str = std::to_string(host_point_data_ip[0]) + "." +
      std::to_string(host_point_data_ip[1]) + "." + std::to_string(host_point_data_ip[2]) + "." +
      std::to_string(host_point_data_ip[3]);
  strcpy(info.host_point_data_ip, host_point_data_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.host_point_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);


  uint8_t host_imu_data_ip[4];
  memcpy(host_imu_data_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_imu_data_ip_str;
  host_imu_data_ip_str = std::to_string(host_imu_data_ip[0]) + "." +
      std::to_string(host_imu_data_ip[1]) + "." + std::to_string(host_imu_data_ip[2]) + "." +
      std::to_string(host_imu_data_ip[3]);
  strcpy(info.host_imu_data_ip, host_imu_data_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.host_imu_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);
}

} // namespace livox
} // namespace direct