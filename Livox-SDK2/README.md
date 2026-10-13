# 1. Introduction

Livox SDK2 is a software development kit designed for all Livox lidars such as HAP and Mid-360. It is developed based on C/C++ following Livox SDK2 Communication Protocol, and provides easy-to-use C style APIs. With the Livox SDK2, users can quickly connect to the Livox Lidars and receive point cloud data.

Livox SDK2 consists of [Livox SDK2 core code](sdk_core/), [Livox SDK2 APIs](include/livox_lidar_api.h) and three [samples](samples/).

## Livox SDK2 API

Livox SDK2 API provides a set of C-style APIs, which can be conveniently integrated in C/C++ programs. Please refer to the **[Livox SDK2 APIs](include/livox_lidar_api.h)**.

## Livox SDK2 Communication Protocol

Livox SDK2 communication protocol opens to all users. It is the communication protocol between user programs and livox products. The protocol consists of control commands and data format, please refer to the documents below:

**Mid-360(S)**:

* [Mid-360(S) Communication protocol](https://livox-wiki-cn.readthedocs.io/zh_CN/latest/tutorials/new_product/mid360/mid360.html) (中文)
* [Mid-360(S) Communication protocol](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html) (English)

**HAP(TX/T1)**:

* [HAP Communication protocol](<https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP>) (中文)
* [HAP Communication protocol](<https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP(English)>) (English)


# 2. Installation

## 2.1 Prerequisites

* OS:
  * Linux: Ubuntu 18.04 or above
  * Windows 10 / 11

* Tools:
  * compilers that support C++11
  * cmake 3.0+

* Arch:
  * x86
  * ARM
## 2.2 Instruction for Ubuntu 20.04

1. Dependencies:

* [CMake 3.0.0+](https://cmake.org/)
* gcc 4.8.1+

2. Install the **CMake** using apt:

```shell
$ sudo apt install cmake
```

3. Compile and install the Livox-SDK2:

```shell
$ git clone https://github.com/Livox-SDK/Livox-SDK2.git
$ cd ./Livox-SDK2/
$ mkdir build
$ cd build
$ cmake .. && make -j
$ sudo make install
```

**Note :**  
The generated shared library and static library are installed to the directory of "/usr/local/lib". The header files are installed to the directory of "/usr/local/include".

Tips: Remove Livox SDK2:

```shell
$ sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
$ sudo rm -rf /usr/local/include/livox_lidar_*
```

## 2.3 Instruction for Windows 10

1. Dependencies:

* Visual Studio 2019
* [CMake 3.0.0+](https://cmake.org/)

2. Preparation:

```cmd
> git clone https://github.com/Livox-SDK/Livox-SDK2.git
> cd Livox-SDK2
> md build && cd build
```

3. Generate a project
* 64-bit project:

```cmd
> cmake .. -G "Visual Studio 16 2019" -A x64
```

* 32-bit project:

```cmd
> cmake .. -G "Visual Studio 16 2019" -A Win32
```

4. Compiling:

You can now compile the Livox-SDK2 in Visual Studio 2019.


# 3. Run the Samples

Livox SDK2 includes three samples, which are "livox_lidar_quick_start", "logger" and "multi_lidars_upgrade".

## 3.1 Livox lidar quick start sample

### In Ubuntu 20.04

Connect to the Lidar(s), and run the program '**livox_lidar_quick_start**' :

```shell
$ cd samples/livox_lidar_quick_start && ./livox_lidar_quick_start ../../../samples/livox_lidar_quick_start/[config file]
```

### In Windows 10
After compiling the Livox SDK2 as shown in Installation above, you can find '**livox_lidar_quick_start.exe**' in the directory of '**Livox-SDK2\\build\\samples\\livox_lidar_quick_start\\Debug(or Release)\\**'.

Copy the config file '**Livox-SDK2\\samples\\livox_lidar_quick_start\\[config file]**' into the directory containing the program '**livox_lidar_quick_start.exe**', and run:


```cmd
> livox_lidar_quick_start.exe [config file]
```

Then you can see the information as below:

```shell
> [info] Data Handle Init Succ.  [data_handler.cpp] [Init] [42]
> [info] Create detection channel detection socket:0  [device_manager.cpp] [CreateDetectionChannel] [232]
```

**Note** : 
1. [config file] in the command above represents the config file name, you can choose different config file depends on your needs.

## 3.2 Logger sample

### Parameters Configuration

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| lidar_log_enable | Enable or disable lidar logger. <br>Logger is enabled by default. | true    |
| lidar_log_cache_size_MB  | Set lidar log cache size. The unit is MB .<br> | 500       |
| lidar_log_path  | Set the save path of lidar log file.<br>The log file is saved in the current path by default.  | "./"       |

These Parameters are located in hap_config.json / mid360_config.json files.

### Run the 'logger' in Ubuntu 20.04

Connect to the Lidar(s), and run the program '**logger**' :

```shell
$ cd samples/logger && ./logger ../../../samples/logger/[config file]
```

### Run the 'logger' in Windows 10

After compiling the Livox SDK2 as shown in Installation above, you can find '**logger.exe**' in the directory of '**Livox-SDK2\\build\\samples\\logger\\Debug(or Release)\\**'.

Copy the config file '**Livox-SDK2\\samples\\logger\\[config file]**' into the directory containing the program '**logger.exe**', and run:


```cmd
> logger.exe [config file]
```

**Note** : 
1. [config file] in the command above represents the config file name, you can choose different config file depends on your needs.

## 3.3 Multi-lidars upgrade sample

### in Ubuntu 20.04

Connect to the Lidar(s), and run the program '**multi_lidars_upgrade**' :

```shell
$ cd samples/multi_lidars_upgrade && ./multi_lidars_upgrade ../../../samples/[config file] [firmware file path]
```
After executing the above command, Lidar stops and the firmware upgrade starts. 
The Lidar(s) upgrade takes a while and the upgrade progress is printed on termial. Also "upgrade successfully" will be printed on terminal when finishing upgrading.

### in Windows 10
After compiling the Livox SDK2 as shown in Installation above, you can find '**multi_lidars_upgrade.exe**' in the directory of '**Livox-SDK2\\build\\samples\\multi_lidars_upgrade\\Debug(or Release)\\**'.

Copy the config file '**Livox-SDK2\\samples\\multi_lidars_upgrade\\[config file]**' and firmware file into the directory containing the program '**multi_lidars_upgrade.exe**', and run:

```cmd
> multi_lidars_upgrade.exe [config file] [firmware file name]
```


**Note** : 
1. [config file] in the command above represents the config file name, you can choose different config file depends on your needs.

# 4. Config file
## 4.1 Basic Configuration
Here is a basic config sample with all REQUIRED fields:
```json
{
  "HAP": {
    "lidar_net_info" : {
      "cmd_data_port"  : 56000,
      "push_msg_port"  : 0,
      "point_data_port": 57000,
      "imu_data_port"  : 58000,
      "log_data_port"  : 59000
    },
    "host_net_info" : [
      {
        "lidar_ip"       : ["192.168.1.10","192.168.1.11","192.168.1.12", "192.168.1.13"],
        "host_ip"        : "192.168.1.5",
        "cmd_data_port"  : 56000,
        "push_msg_port"  : 0,
        "point_data_port": 57000,
        "imu_data_port"  : 58000,
        "log_data_port"  : 59000
      }
    ]
  }
}
```
### Description for REQUIRED fields  
* "HAP": Lidar type, meaning the following configuration is for HAP lidar type; Another option is "MID360", for configuration of MID-360 lidar type.
  * "lidar_net_info": set the ports in the lidar.
    * "cmd_data_port": port for sending / receiving control command.
    * "push_msg_port": port for sending push message.
    * "point_data_port": port for sending point cloud data.
    * "imu_data_port": port for sending imu data.
    * "log_data_port": port for sending firmware log data.
  * "host_net_info": set the configuration of the host machines, and the value is a list, meaning that you can configure several hosts.
    * "lidar_ip": this is a list, indicating all ips of the lidars intended to connect to this host.
    * "host_ip": the ip of the host you're configuring.
    * "cmd_data_port": port for sending / receiving control command.
    * "push_msg_port" port for receiving push message.
    * "point_data_port": port for receiving point cloud data.
    * "imu_data_port": port for receiving imu data.
    * "log_data_port": port for receiving firmware log data.


## 4.2 Full Configuration
Here is a full sample including multi-lidar types configurations and some OPTIONAL fields:
```json
{
  "master_sdk" : true,
  "lidar_log_enable"        : true,
  "lidar_log_cache_size_MB" : 500,
  "lidar_log_path"          : "./",

  "HAP": {
    "lidar_net_info" : {
      "cmd_data_port"  : 56000,
      "push_msg_port"  : 0,
      "point_data_port": 57000,
      "imu_data_port"  : 58000,
      "log_data_port"  : 59000
    },
    "host_net_info" : [
      {
        "lidar_ip"       : ["192.168.1.10","192.168.1.11","192.168.1.12", "192.168.1.13"],
        "host_ip"        : "192.168.1.5",
        "multicast_ip"   : "224.1.1.5",
        "cmd_data_port"  : 56000,
        "push_msg_port"  : 0,
        "point_data_port": 57000,
        "imu_data_port"  : 58000,
        "log_data_port"  : 59000
      }
    ]
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port"  : 56100,
      "push_msg_port"  : 56200,
      "point_data_port": 56300,
      "imu_data_port"  : 56400,
      "log_data_port"  : 56500
    },
    "host_net_info" : [
      {
        "lidar_ip"       : ["192.168.1.3"],
        "host_ip"        : "192.168.1.5",
        "multicast_ip"   : "224.1.1.5",
        "cmd_data_port"  : 56101,
        "push_msg_port"  : 56201,
        "point_data_port": 56301,
        "imu_data_port"  : 56401,
        "log_data_port"  : 56501
      }
    ]
  }
}
```
### Description for OPTIONAL fields
* "master_sdk": used in multi-casting scenario. 
  * 'true' stands for master SDK and 'false' stands for slave SDK;
  * 'master SDK' can send control command to and receive data from the lidars, while 'slave SDK' can only receive point cloud data from the lidars.
  * NOTICE: ONLY ONE SDK (host) can be set as 'master SDK'. Others should be set as 'slave SDK'.
* "lidar_log_enable": 'true' or 'false' represents whether to enable the firmware log.
* "lidar_log_cache_size_MB": set the storage size for firmware log, unit: MB.
* "lidar_log_path": set the path to store the firmware log data.
* "multicast_ip": this field is in the parent key "host_net_info", representing the multi-casting IP.

# 5. Support

You can get support from Livox via:

* Send an email to cs@livoxtech.com, appended with detailed description for your problem and your setup;
* Raise a github issue

