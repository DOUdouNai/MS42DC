#include "MS42DC.h"

// 定义串口对象
HardwareSerial MS42DC_Serial(1); // 使用 UART1，根据您的硬件连接选择 UART0 或 UART1

// 计算 BCC 校验位的函数
uint8_t calculateBCC(const uint8_t* data, size_t length) {
    uint8_t bcc = 0;
    for (size_t i = 0; i < length; ++i) {
        bcc ^= data[i];
    }
    return bcc;
}

// 构建命令数组的函数
void buildCommand(uint8_t deviceAddress, uint8_t controlMode, uint8_t direction, int16_t value, uint8_t* command) {
    uint16_t dataValue = abs(value);
    uint8_t dataHigh = (dataValue >> 8) & 0xFF;
    uint8_t dataLow = dataValue & 0xFF;
    
    uint16_t speed = DEFAULT_SPEED;
    uint8_t speedHigh = (speed >> 8) & 0xFF;
    uint8_t speedLow = speed & 0xFF;
    
    // 构建命令数组
    command[0] = FRAME_HEADER;
    command[1] = deviceAddress;
    command[2] = controlMode;
    command[3] = direction;
    command[4] = SUBDIVISION_32;
    
    if (controlMode == CONTROL_MODE_POSITION || controlMode == CONTROL_MODE_ABSOLUTE_ANGLE) {
        command[5] = dataHigh;
        command[6] = dataLow;
    } else if (controlMode == CONTROL_MODE_TORQUE) {
        command[5] = dataHigh;
        command[6] = dataLow;
    } else if (controlMode == CONTROL_MODE_SPEED) {
        command[5] = speedHigh;
        command[6] = speedLow;
    }
    
    // 速度数据在位置控制模式和力矩控制模式下为0
    // if (controlMode == CONTROL_MODE_POSITION || controlMode == CONTROL_MODE_TORQUE || controlMode == CONTROL_MODE_ABSOLUTE_ANGLE) {
    //     command[7] = 0x00;
    //     command[8] = 0x00;
    // } else {
    //     command[7] = speedHigh;
    //     command[8] = speedLow;
    // }
    command[7] = speedHigh;
    command[8] = speedLow;
    // 计算 BCC 校验位
    command[9] = calculateBCC(command, 9);
    command[10] = FRAME_TAIL;
}

// 发送命令的函数
void sendCommand(uint8_t deviceAddress, uint8_t controlMode, uint8_t direction, int16_t value) {
    uint8_t command[11];
    buildCommand(deviceAddress, controlMode, direction, value, command);
    MS42DC_Serial.write(command, sizeof(command));
}

// 初始化串口通信的函数
void initializeSerial(uint32_t baudRate) {
    MS42DC_Serial.begin(baudRate, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(1000); // 等待串口初始化
    MS42DC_Serial.println("MS42DC 串口初始化完成");
}

// 速度控制模式的实现
void speedControl(uint8_t deviceAddress, uint8_t direction, uint16_t speed) {
    sendCommand(deviceAddress, CONTROL_MODE_SPEED, direction, speed);
    Serial.println("速度控制模式命令已发送");
}

// 位置控制模式的实现
void positionControl(uint8_t deviceAddress, uint8_t direction, int16_t position) {
    sendCommand(deviceAddress, CONTROL_MODE_POSITION, direction, position);
    Serial.println("位置控制模式命令已发送");
}

// 力矩控制模式的实现
void torqueControl(uint8_t deviceAddress, uint8_t direction, uint16_t torque) {
    sendCommand(deviceAddress, CONTROL_MODE_TORQUE, direction, torque);
    Serial.println("力矩控制模式命令已发送");
}

// 绝对角度控制模式的实现
void absoluteAngleControl(uint8_t deviceAddress, uint8_t direction, int16_t angle) {
    sendCommand(deviceAddress, CONTROL_MODE_ABSOLUTE_ANGLE, direction, angle);
    Serial.println("绝对角度控制模式命令已发送");
}