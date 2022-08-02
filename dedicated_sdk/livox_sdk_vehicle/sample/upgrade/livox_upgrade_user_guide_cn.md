## 1 升级工具使用说明

### 1.1 编译 SDK 以及 SDK upgrade sample

livox_upgrade 以 SDK sample 的方式提供给用户参考使用，源代码位于 XP_HAP_SDK/hap_sdk/sample/upgrade 目录下。hap_sdk 工程编译时，upgrade sample 子工程被一起编译并生成可执行文件。编译 hap_sdk 使用如下命令：

```
cd XP_HAP_SDK/hap_sdk/

mkdir build && cd build

cmake ..

make
```

### 1.2 运行 livox_upgrade

运行 livox_upgrade 的命令格式如下：

```
./livox_upgrade [broadcast_code] [firmware_path]
```

举例，假设具体参数如下：

（1）当前 Hap 的广播码是 0TFDG3B006H2Z11；

（2）待升级固件名是 LIVOX_HAP_FW_15.02.0016.bin，位于 home 目录；

（3）位于 build 目录；

相应的升级命令如下：

```
./smaple/upgrade/livox_upgrade 0TFDG3B006H2Z11 /home/LIVOX_HAP_FW_15.02.0016.bin
```

运行如上命令后，程序会自动给指定的雷达进行固件升级，升级结束后程序会自动退出。

### 1.3 配置升级程序

第一次使用编译 livox_upgrade 前，需要配置主机网卡的 IP 地址、待升级 lidar 的 slot id 和 IP 地址，源代码位于 livox_upgrade_tool.cpp中，如下：

```
45 const char* kNetIf = "192.168.1.6"; // local netcard's address
46 const LidarRegisterInfo lidar_info[kLidarNumber] = {
47  { 1, "192.168.1.4" },
48  { 2, "172.20.1.53" }
49 };
```

