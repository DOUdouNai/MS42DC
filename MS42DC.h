#ifndef MS42DC_H
#define MS42DC_H

#include <Arduino.h>
#include <HardwareSerial.h>

// ===========================
// ======= 配置部分 ==========
// ===========================

// 定义串口引脚（根据您的硬件连接进行修改）
const int RX_PIN = 16; // 接收引脚（根据您的ESP32板子进行设置）
const int TX_PIN = 17; // 发送引脚（根据您的ESP32板子进行设置）

// 创建串口对象
extern HardwareSerial MS42DC_Serial;

// ===========================
// ======= 常量定义 ==========
// ===========================

// 帧头和帧尾
const uint8_t FRAME_HEADER = 0x7B; // '{'
const uint8_t FRAME_TAIL = 0x7D;   // '}'

// 细分值定义
const uint8_t SUBDIVISION_32 = 0x20; // 32细分

// 默认速度（单位：rad/s，放大10倍）
const uint16_t DEFAULT_SPEED = 100; // 10.0 rad/s

// 控制模式定义
const uint8_t CONTROL_MODE_SPEED = 0x01;            // 速度控制模式
const uint8_t CONTROL_MODE_POSITION = 0x02;         // 位置控制模式
const uint8_t CONTROL_MODE_TORQUE = 0x03;           // 力矩控制模式
const uint8_t CONTROL_MODE_ABSOLUTE_ANGLE = 0x04;   // 单圈绝对角度控制模式

// 转向定义
const uint8_t DIRECTION_CW = 0x01;  // 顺时针
const uint8_t DIRECTION_CCW = 0x00; // 逆时针

// ===========================
// ======= 函数声明 ==========
// ===========================

/**
 * @brief 初始化串口通信
 * 
 * @param baudRate 波特率（默认115200）
 */
void initializeSerial(uint32_t baudRate = 115200);

/**
 * @brief 计算BCC校验位
 * 
 * @param data 数据数组
 * @param length 数据长度
 * @return uint8_t BCC校验位
 */
uint8_t calculateBCC(const uint8_t* data, size_t length);

/**
 * @brief 构建命令数组
 * 
 * @param deviceAddress 设备地址
 * @param controlMode 控制模式
 * @param direction 转向
 * @param value 值（角度或电流）
 * @param command 命令数组
 */
void buildCommand(uint8_t deviceAddress, uint8_t controlMode, uint8_t direction, int16_t value, uint8_t* command);

/**
 * @brief 发送命令
 * 
 * @param deviceAddress 设备地址
 * @param controlMode 控制模式
 * @param direction 转向
 * @param value 值（角度或电流）
 */
void sendCommand(uint8_t deviceAddress, uint8_t controlMode, uint8_t direction, int16_t value);

/**
 * @brief 速度控制模式
 * 
 * @param deviceAddress 设备地址
 * @param direction 转向
 * @param speed 速度（单位：rad/s，放大10倍）
 */
void speedControl(uint8_t deviceAddress, uint8_t direction, uint16_t speed);

/**
 * @brief 位置控制模式
 * 
 * @param deviceAddress 设备地址
 * @param direction 转向
 * @param position 目标位置（单位：度）
 */
void positionControl(uint8_t deviceAddress, uint8_t direction, int16_t position);

/**
 * @brief 力矩控制模式
 * 
 * @param deviceAddress 设备地址
 * @param direction 转向
 * @param torque 力矩值（单位：mA）
 */
void torqueControl(uint8_t deviceAddress, uint8_t direction, uint16_t torque);

/**
 * @brief 绝对角度控制模式
 * 
 * @param deviceAddress 设备地址
 * @param direction 转向
 * @param angle 目标角度（单位：度）
 */
void absoluteAngleControl(uint8_t deviceAddress, uint8_t direction, int16_t angle);

#endif // MS42DC_H