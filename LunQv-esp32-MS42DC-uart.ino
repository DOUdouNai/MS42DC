#include "MS42DC.h"

const uint8_t MOTOR_ID = 0x02; // 根据您的电机ID进行调整


void setup() {
    // 初始化默认串口通信
    Serial.begin(115200);
    delay(1000); // 等待串口初始化完成
    Serial.println("开始执行 setup()");

    // 初始化 MS42DC 串口通信
    initializeSerial(115200);
    delay(1000); // 等待 MS42DC 串口初始化完成
    Serial.println("MS42DC 串口初始化完成");

    // // // 测试速度控制模式
    // speedControl(0x07, DIRECTION_CW, 100); // 顺时针，10.0 rad/s
    // delay(2000); // 运行2秒

    // speedControl(0x03, DIRECTION_CCW, 50); // 逆时针，5.0 rad/s
    // delay(2000); // 运行2秒

    // // // 测试位置控制模式
    // positionControl(0x04, DIRECTION_CW, 3600); // 顺时针转动360度
    // delay(3000); // 等待电机完成转动
    // positionControl(0x02, DIRECTION_CW, 720); // 顺时针转动360度
    // delay(3000); // 等待电机完成转动


    // positionControl(MOTOR_ID, DIRECTION_CCW, 180); // 逆时针转动180度
    // delay(3000); // 等待电机完成转动

    // positionControl(MOTOR_ID, DIRECTION_CW, 720); // 顺时针转动720度
    // delay(5000); // 等待电机完成转动

    // // 测试绝对角度控制模式
    // absoluteAngleControl(MOTOR_ID, DIRECTION_CW, 100); // 转动到100度
    // delay(2000); // 等待电机完成转动

    // absoluteAngleControl(MOTOR_ID, DIRECTION_CCW, 200); // 转动到200度
    // delay(2000); // 等待电机完成转动

    // absoluteAngleControl(MOTOR_ID, DIRECTION_CW, 0); // 转动到0度
    // delay(2000); // 等待电机完成转动

    // // 测试力矩控制模式
    // torqueControl(MOTOR_ID, DIRECTION_CW, 500); // 顺时针，500 mA
    // delay(2000); // 运行2秒

    // torqueControl(MOTOR_ID, DIRECTION_CCW, 800); // 逆时针，800 mA
    // delay(2000); // 运行2秒

    // torqueControl(MOTOR_ID, DIRECTION_CW, 1000); // 顺时针，1000 mA
    // delay(2000); // 运行2秒

    // torqueControl(MOTOR_ID, DIRECTION_CW, 0); // 停止电机
    // Serial.println("所有测试完成");
    // absoluteAngleControl(0x04, DIRECTION_CW, 100); // 顺时针，10.0 rad/s
    // absoluteAngleControl(0x07, DIRECTION_CW, 100); // 顺时针，10.0 rad/s
    // absoluteAngleControl(0x02, DIRECTION_CW, 100); // 顺时针，10.0 rad/s
    // absoluteAngleControl(0x03, DIRECTION_CW, 100); // 顺时针，10.0 rad/s
    // delay(10000); // 运行2秒
    // absoluteAngleControl(0x04, DIRECTION_CCW, 100); // 顺时针，10.0 rad/s
    // absoluteAngleControl(0x07, DIRECTION_CCW, 100); // 顺时针，10.0/ rad/s
    // absoluteAngleControl(0x02, DIRECTION_CCW, 100); // 顺时针，10.0 rad/s
    // absoluteAngleControl(0x03, DIRECTION_CCW, 100); // 顺时针，10.0 rad/s
}

void loop() {
    // 主循环中无需执行任何操作
    // Serial.println("主循环执行中...");
    delay(1000);
    // 测试速度控制模式
    // absoluteAngleControl(0x04, DIRECTION_CW, 0); // 顺时针，10.0 rad/s
    // absoluteAngleControl(0x07, DIRECTION_CW, 0); // 顺时针，10.0 rad/s
    // absoluteAngleControl(0x02, DIRECTION_CW, 0); // 顺时针，10.0 rad/s
    // absoluteAngleControl(0x03, DIRECTION_CW, 0); // 顺时针，10.0 rad/s
    // delay(5000); // 运行2秒
    // positionControl(0x04, DIRECTION_CW, 1000); // 顺时针，10.0 rad/s
    // positionControl(0x07, DIRECTION_CW, 1000); // 顺时针，10.0 rad/s
    // positionControl(0x02, DIRECTION_CW, 1000); // 顺时针，10.0 rad/s
    // positionControl(0x03, DIRECTION_CW, 1000); // 顺时针，10.0 rad/s
    // delay(5000); // 运行2秒
    // // 测试位置控制模式
    positionControl(0x04, DIRECTION_CW, 3600); // 顺时针转动360度
    delay(3000); // 等待电机完成转动
    positionControl(0x02, DIRECTION_CW, 1000); // 顺时针转动360度
    delay(3000); // 等待电机完成转动

}