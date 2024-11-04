# Battery-powered-LD2410S-sensor

*Project*: Human Presence Sensor with LD2410S

*Hardware*: Lolin 32 Lite 

*Framework*: Arduino IDE

## Description
这是一个基于ESP32和LD2410S毫米波雷达传感器的人体存在检测设备。
设备具有以下功能：
1. 自动配置LD2410S传感器的检测门限
2. WiFi连接和网络配置（使用WiFiManager）
3. MQTT消息推送（使用巴法云平台）
4. 低功耗设计（使用深度睡眠模式）
5. LED状态指示
6. 电池电压监测

## Hardware Connections
- LD2410S TX -> ESP32 GPIO25 (RX)
- LD2410S RX -> ESP32 GPIO26 (TX)
- LD2410S OUT -> ESP32 GPIO27
- LED -> ESP32 GPIO22
- BAT_ADC -> ESP32 GPIO35

## Dependent Libraries
- WiFi.h
- WiFiManager.h
- HTTPClient.h
- esp_sleep.h

## Usage Instructions
1. 首次上电会进入自动配置模式，LED指示灯会显示配置进度
2. 如果是首次连接WiFi，设备会创建AP热点"ESP32_HumanSensor"
3. 连接该热点并配置WiFi信息
4. 正常工作后，检测到人体存在时会通过MQTT发送状态
5. 设备支持深度睡眠以节省电量

