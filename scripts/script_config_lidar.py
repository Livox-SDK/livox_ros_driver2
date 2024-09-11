## O script tem a funcionalidade de criar o arquivo .json de forma mais rapida e pratica; Um exemplo do .json criado é o "final_config.json" 

import os
import json

# Definições das variáveis
host_ip = "192.168.1.42"

## Config HAP
cmd_data_port_HAP = 56000
push_msg_port_HAP = 0
point_data_port_HAP = 57000
imu_data_port_HAP = 58000
log_data_port_HAP = 59000

## Config MID360
cmd_data_port_MID360 = 56100
host_cmd_data_port_MID360 = 56101
push_msg_port_MID360 = 56200
host_push_msg_port_MID360 = 56201
point_data_port_MID360 = 56300
host_point_data_port_MID360 = 56301
imu_data_port_MID360 = 56400
log_data_port_MID360 = 56500
host_imu_data_port_MID360 = 56401
host_log_data_port_MID360 = 56501

default_extrinsics_hap = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "x": 0,
    "y": 0,
    "z": 0
}
default_extrinsics_front ={
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "x": -20,
    "y": 200,
    "z": 0
}
default_extrinsics_rear = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 180.0,
    "x": -650,
    "y": -300,
    "z": 0
}


format_config = {
    "lidar_summary_info": {
        "lidar_type": 8
    },
    "HAP": {
        "lidar_net_info": {
            "cmd_data_port": cmd_data_port_HAP,
            "push_msg_port": push_msg_port_HAP,
            "point_data_port": point_data_port_HAP,
            "imu_data_port": imu_data_port_HAP,
            "log_data_port": log_data_port_HAP
        },
        "host_net_info": {
            "cmd_data_ip": host_ip,
            "cmd_data_port": cmd_data_port_HAP,
            "push_msg_ip": "",
            "push_msg_port": push_msg_port_HAP,
            "point_data_ip": host_ip,
            "point_data_port": point_data_port_HAP,
            "imu_data_ip": host_ip,
            "imu_data_port": imu_data_port_HAP,
            "log_data_ip": "",
            "log_data_port": log_data_port_HAP
        }
    },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": cmd_data_port_MID360,
            "push_msg_port": push_msg_port_MID360,
            "point_data_port": point_data_port_MID360,
            "imu_data_port": imu_data_port_MID360,
            "log_data_port": log_data_port_MID360
        },
        "host_net_info": {
            "cmd_data_ip": host_ip,
            "cmd_data_port": host_cmd_data_port_MID360,
            "push_msg_ip": host_ip,
            "push_msg_port": host_push_msg_port_MID360,
            "point_data_ip": host_ip,
            "point_data_port": host_point_data_port_MID360,
            "imu_data_ip": host_ip,
            "imu_data_port": host_imu_data_port_MID360,
            "log_data_ip": "",
            "log_data_port": host_log_data_port_MID360
        }
    },
    "lidar_configs": [
        {
            "ip": "192.168.1.100",
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": default_extrinsics_hap
        },
        {
            "ip": "192.168.1.125",
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": default_extrinsics_front
        },
        {   
            "ip": "192.168.1.128",
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": default_extrinsics_rear
          }
      ]
}

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
# cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_path, 'final_config.json')

# Verifica se o arquivo já existe
if not os.path.exists(user_config_path):
    with open(user_config_path, 'w') as json_file:
        json.dump(format_config, json_file, indent=2)
    # print(f"Arquivo salvo em: {user_config_path}")