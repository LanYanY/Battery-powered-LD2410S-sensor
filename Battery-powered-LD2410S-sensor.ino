/*
 * Project: Human Presence Sensor with LD2410S
 * Hardware: ESP32-WROOM-32
 * Framework: Arduino IDE
 * Author: 三灶
 * Date: 2024
 * 
 * Description:
 * 这是一个基于ESP32和LD2410S毫米波雷达传感器的人体存在检测设备。
 * 设备具有以下功能：
 * 1. 自动配置LD2410S传感器的检测门限
 * 2. WiFi连接和网络配置（使用WiFiManager）
 * 3. MQTT消息推送（使用巴法云平台）
 * 4. 低功耗设计（使用深度睡眠模式）
 * 5. LED状态指示
 * 6. 电池电压监测
 * 
 * 硬件连接：
 * - LD2410S TX -> ESP32 GPIO25 (RX)
 * - LD2410S RX -> ESP32 GPIO26 (TX)
 * - LD2410S OUT -> ESP32 GPIO27
 * - LED -> ESP32 GPIO22
 * - BAT_ADC -> ESP32 GPIO35
 * 
 * 依赖库：
 * - WiFi.h
 * - WiFiManager.h
 * - HTTPClient.h
 * - esp_sleep.h
 * 
 * 使用说明：
 * 1. 首次上电会进入自动配置模式，LED指示灯会显示配置进度
 * 2. 如果是首次连接WiFi，设备会创建AP热点"ESP32_HumanSensor"
 * 3. 连接该热点并配置WiFi信息
 * 4. 正常工作后，检测到人体存在时会通过MQTT发送状态
 * 5. 设备支持深度睡眠以节省电量
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <esp_sleep.h>

#define DEBUG 0

// 巴法云配置
#define UID "your uid" // 用户私钥
const char *topic = "LD2410S007";              // 主题名字
const char *api_url = "http://apis.bemfa.com/va/postmsg";

// 引脚定义
const int SENSOR_PIN = 27;    // LD2410S 传感器输入引脚
const int LED_PIN = 22;       // LED指示灯引脚
const int BATTERY_PIN = 35;   // 电池电压检测引脚
const int RX_PIN = 25;        // LD2410S UART RX引脚
const int TX_PIN = 26;        // LD2410S UART TX引脚

// 全局变量
RTC_DATA_ATTR int bootCount = 0; // 存储在RTC内存中的启动计数
RTC_DATA_ATTR uint32_t static_ip = 0;   // 固定IP
RTC_DATA_ATTR uint32_t gate_way = 0;    // 网关
RTC_DATA_ATTR uint32_t sub_net = 0;     // 子网掩码
RTC_DATA_ATTR uint32_t dns_server = 0;  // DNS服务器
RTC_DATA_ATTR unsigned long lastWakeupDuration = 0;  // 上次唤醒持续时间(毫秒)
RTC_DATA_ATTR unsigned long wakeupStartTime = 0;     // 本次唤醒开始时间

// 添加MQTT相关常量
#define MQTT_SERVER "bemfa.com"
#define MQTT_PORT 9501
#define MQTT_KEEP_ALIVE 60
#define MQTT_BUFFER_SIZE 256

// 添加缺失的常量和变声明
#define WIFI_TIMEOUT_MS 10000  // WiFi连接超时时间（毫秒）

// 添加新的常量
const unsigned long AUTO_CONFIG_TIMEOUT = 130 * 1000;  // 自动配置超时时间(130秒)
const int SLOW_BLINK_MS = 1000;   // 慢闪周期
const int FAST_BLINK_MS = 200;    // 快闪周期

// 添加自动配置命令数组
const uint8_t CMD1[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x02, 0x00, 0x04, 0x03, 0x02, 0x01};
const uint8_t CMD2[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x02, 0x00, 0x04, 0x03, 0x02, 0x01};
const uint8_t CMD3[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x09, 0x00, 0x02, 0x00, 0x01, 0x00, 0x78, 0x00, 0x04, 0x03, 0x02, 0x01};
const uint8_t CMD4[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};

// 时间记录变量
unsigned long timeWiFiStart;
unsigned long timeWiFiEnd;
unsigned long timeHttpStart;
unsigned long timeHttpEnd;

// 添加MQTT客户端类
class MiniMQTT {
private:
    WiFiClient client;
    uint16_t messageId = 0;
    
    uint16_t writeString(const char* string, uint8_t* buf) {
        uint16_t length = strlen(string);
        buf[0] = length >> 8;
        buf[1] = length & 0xFF;
        memcpy(buf + 2, string, length);
        return length + 2;
    }

public:
    bool connect(const char* clientId) {
        if (!client.connect(MQTT_SERVER, MQTT_PORT)) {
            return false;
        }

        uint8_t buffer[MQTT_BUFFER_SIZE];
        uint16_t pos = 0;

        // Fixed header
        buffer[pos++] = 0x10;  // CONNECT type
        
        // Remaining length placeholder
        pos++;  
        
        // Protocol name
        pos += writeString("MQTT", buffer + pos);
        
        // Protocol level
        buffer[pos++] = 0x04;  // MQTT 3.1.1
        
        // Connect flags
        buffer[pos++] = 0x02;  // Clean session
        
        // Keep alive
        buffer[pos++] = MQTT_KEEP_ALIVE >> 8;
        buffer[pos++] = MQTT_KEEP_ALIVE & 0xFF;
        
        // Client ID
        pos += writeString(clientId, buffer + pos);
        
        // Update remaining length
        buffer[1] = pos - 2;
        
        client.write(buffer, pos);
        
        // 等待CONNACK
        unsigned long timeout = millis() + 5000;
        while (client.available() < 4 && millis() < timeout) {
            delay(10);
        }
        
        if (client.available() < 4) {
            client.stop();
            return false;
        }
        
        client.read();  // 第一个字节
        client.read();  // 剩余长度
        client.read();  // 连接确认标志
        return client.read() == 0x00;  // 返回码
    }

    bool publish(const char* topic, const char* payload) {
        if (!client.connected()) {
            return false;
        }

        uint16_t payloadLength = strlen(payload);
        uint16_t topicLength = strlen(topic);
        uint16_t remainingLength = 2 + topicLength + payloadLength;

        // Fixed header
        client.write(0x30);  // PUBLISH type
        client.write(remainingLength);
        
        // Topic
        client.write(topicLength >> 8);
        client.write(topicLength & 0xFF);
        client.write((uint8_t*)topic, topicLength);
        
        // Payload
        client.write((uint8_t*)payload, payloadLength);
        
        return true;
    }

    void disconnect() {
        if (client.connected()) {
            client.write((uint8_t*)"\xe0\x00", 2);
            client.flush();
            client.stop();
        }
    }
};

// 读取电池电压的函数
float getBatteryVoltage() {
    // 设置ADC衰减为11dB (0-2.6V)
    analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
    // 直接读取毫伏值
    int milliVolt = analogReadMilliVolts(BATTERY_PIN);
    // 由于使用分压电路，实际电压是测量电压的2倍，并转换为伏特
    return (milliVolt * 2.0) / 1000.0;
}

// 添加以下代码到全局变量区域
TaskHandle_t blinkTaskHandle = NULL;
QueueHandle_t ledControlQueue = NULL;

// 定义LED控制消息结构
struct LEDControlMessage {
    enum Mode {
        SLOW_BLINK,    // 慢闪
        FAST_BLINK,    // 快闪
        PROGRESS_BLINK,// 进度闪烁
        SOLID_ON,      // 常亮
        SOLID_OFF      // 常灭
    } mode;
    int progress;      // 用于进度闪烁模式
};

// 添加LED控制任务函数
void ledControlTask(void *parameter) {
    LEDControlMessage msg;
    unsigned long lastToggle = 0;
    bool ledState = false;
    
    while(1) {
        if(xQueueReceive(ledControlQueue, &msg, 0) == pdTRUE) {
            // 收到新的控制消息时重置状态
            lastToggle = 0;
            ledState = false;
        }
        
        unsigned long now = millis();
        
        switch(msg.mode) {
            case LEDControlMessage::SLOW_BLINK:
                if(now - lastToggle >= 1000) {  // 1秒翻转一次
                    ledState = !ledState;
                    digitalWrite(LED_PIN, ledState);
                    lastToggle = now;
                }
                break;
                
            case LEDControlMessage::FAST_BLINK:
                if(now - lastToggle >= 200) {   // 0.2秒翻转一次
                    ledState = !ledState;
                    digitalWrite(LED_PIN, ledState);
                    lastToggle = now;
                }
                break;
                
            case LEDControlMessage::PROGRESS_BLINK:
                {
                    // 根据进度计算闪烁间隔（500ms到50ms）
                    int interval = 500 - (msg.progress * 450 / 100);
                    if(now - lastToggle >= interval) {
                        ledState = !ledState;
                        digitalWrite(LED_PIN, ledState);
                        lastToggle = now;
                    }
                }
                break;
                
            case LEDControlMessage::SOLID_ON:
                digitalWrite(LED_PIN, HIGH);
                break;
                
            case LEDControlMessage::SOLID_OFF:
                digitalWrite(LED_PIN, LOW);
                break;
        }
        
        vTaskDelay(10);  // 释放一些CPU时间
    }
}

// 修改configureLD2410S()函数
void configureLD2410S() {
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    
    // 创建LED控制队列和任务
    ledControlQueue = xQueueCreate(5, sizeof(LEDControlMessage));
    xTaskCreate(
        ledControlTask,
        "LED_Control",
        2048,
        NULL,
        1,
        &blinkTaskHandle
    );
    
    // 等待传感器稳定，使用慢闪
    LEDControlMessage msg = {LEDControlMessage::SLOW_BLINK, 0};
    xQueueSend(ledControlQueue, &msg, 0);
    
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        while (Serial1.available()) {
            Serial1.read();
        }
        delay(10);
    }

    #ifdef DEBUG
        Serial.println("开始发送自动配置命令...");
    #endif

    // 发送配置命令前切换到快闪
    msg.mode = LEDControlMessage::FAST_BLINK;
    xQueueSend(ledControlQueue, &msg, 0);
    delay(1000);  // 快闪1秒

    // 发送配置命令
    Serial1.write(CMD1, sizeof(CMD1));
    delay(100);
    Serial1.write(CMD2, sizeof(CMD2));
    delay(100);
    Serial1.write(CMD3, sizeof(CMD3));
    delay(100);
    Serial1.write(CMD4, sizeof(CMD4));

    #ifdef DEBUG
        Serial.println("配置命令已发送，等待进度数据...");
    #endif

    // 等待并处理配置进度数据
    startTime = millis();
    uint8_t buffer[13];
    bool configComplete = false;
    int lastProgress = 0;
    
    while (!configComplete && (millis() - startTime < AUTO_CONFIG_TIMEOUT)) {
        while (Serial1.available() > 0) {
            uint8_t byte = Serial1.read();
            
            if (byte == 0xF4) {
                buffer[0] = byte;
                unsigned long waitStart = millis();
                while (Serial1.available() < 12 && millis() - waitStart < 1000) {
                    delay(10);
                }
                
                if (Serial1.available() >= 12) {
                    Serial1.readBytes(buffer + 1, 12);
                    
                    if (buffer[0] == 0xF4 && buffer[1] == 0xF3 && 
                        buffer[2] == 0xF2 && buffer[3] == 0xF1 &&
                        buffer[9] == 0xF8 && buffer[10] == 0xF7 && 
                        buffer[11] == 0xF6 && buffer[12] == 0xF5) {
                        
                        int progress = (buffer[8] << 8) | buffer[7];
                        
                        if (progress != lastProgress) {
                            lastProgress = progress;
                            #ifdef DEBUG
                                Serial.printf("自动门限配置进度: %d%%\n", progress);
                            #endif
                            
                            // 更新LED闪烁进度
                            msg.mode = LEDControlMessage::PROGRESS_BLINK;
                            msg.progress = progress;
                            xQueueSend(ledControlQueue, &msg, 0);
                        }
                        
                        if (progress >= 100) {
                            configComplete = true;
                            // 配置完成后LED常亮3秒
                            msg.mode = LEDControlMessage::SOLID_ON;
                            xQueueSend(ledControlQueue, &msg, 0);
                            delay(3000);
                            msg.mode = LEDControlMessage::SOLID_OFF;
                            xQueueSend(ledControlQueue, &msg, 0);
                            break;
                        }
                    }
                }
            }
        }
        delay(10);
    }

    // 删除LED控制任务和队列
    if(blinkTaskHandle != NULL) {
        vTaskDelete(blinkTaskHandle);
        blinkTaskHandle = NULL;
    }
    if(ledControlQueue != NULL) {
        vQueueDelete(ledControlQueue);
        ledControlQueue = NULL;
    }

    Serial1.end();
    
    #ifdef DEBUG
        if (configComplete) {
            Serial.println("自动门限配置完成");
        } else {
            Serial.println("自动门限配置超时");
        }
    #endif
}

void setup()
{
    // 增加启动计数
    bootCount++;
    
    wakeupStartTime = millis();
    
    #ifdef DEBUG
        Serial.begin(115200);
        Serial.printf("\n\n====== 唤醒次数: %d ======\n", bootCount);
        Serial.printf("CPU频率: %dMHz\n", getCpuFrequencyMhz());
    #endif

    // 配置引脚
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    pinMode(SENSOR_PIN, INPUT);
    int sensorState = digitalRead(SENSOR_PIN);
    
    // 第一次启动时进行自动门限配置
    if (bootCount == 1) {
        configureLD2410S();
    }

    // WiFi连接优化
    timeWiFiStart = millis();
    if (bootCount == 1) {
        WiFiManager wifiManager;
        wifiManager.setConnectTimeout(WIFI_TIMEOUT_MS/1000);
        wifiManager.setConfigPortalTimeout(30);
        wifiManager.autoConnect("ESP32_HumanSensor");
        
        static_ip = (uint32_t)WiFi.localIP();
        gate_way = (uint32_t)WiFi.gatewayIP();
        sub_net = (uint32_t)WiFi.subnetMask();
        dns_server = (uint32_t)WiFi.dnsIP();
    } else {
        WiFi.mode(WIFI_STA);
        WiFi.config(IPAddress(static_ip), IPAddress(gate_way), 
                   IPAddress(sub_net), IPAddress(dns_server));
        WiFi.begin();
    }

    // WiFi连接等待优化
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && 
           millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
        delay(50);
    }
    timeWiFiEnd = millis();
    
    #ifdef DEBUG
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("WiFi连接时间: %ldms\n", timeWiFiEnd - timeWiFiStart);
        }
    #endif

    // 替换HTTP请求部分为MQTT
    if (WiFi.status() == WL_CONNECTED) {
        float batteryVoltage = getBatteryVoltage();
        
        timeHttpStart = millis();
        MiniMQTT mqtt;
        
        if (mqtt.connect(UID)) {
            String payload = String(sensorState == HIGH ? "on" : "off");
            payload += "#";
            payload += String(batteryVoltage, 2);
            payload += "#";
            payload += String(lastWakeupDuration);
            
            mqtt.publish(topic, payload.c_str());
            mqtt.disconnect();
        }
        
        timeHttpEnd = millis();
        
        #ifdef DEBUG
            Serial.printf("MQTT发布时间: %ldms\n", timeHttpEnd - timeHttpStart);
        #endif
    }

    // 计算唤醒时间并准备休眠
    lastWakeupDuration = millis() - wakeupStartTime;
    
    #ifdef DEBUG
        Serial.printf("总唤醒时间: %ldms\n", lastWakeupDuration);
    #endif

    // 断开WiFi连接
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    
    esp_sleep_enable_ext0_wakeup((gpio_num_t)SENSOR_PIN, !sensorState);
    digitalWrite(LED_PIN, HIGH);
    esp_deep_sleep_start();
}

void loop()
{
    // 深度睡眠模式下不会执行loop
}
