#ifndef AVATAR_SERVO_H
#define AVATAR_SERVO_H

#include <Arduino.h>
#include <Wire.h>
#include <String.h>
#include <sstream>
#define GET_LOW_BYTE(A) (uint8_t)((A))                           // 宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)                     // 宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B)) // 宏函数 以A为高adServoPosition八位 B为低八位 合并为16位整形

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels, 32 as default.
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


class Avatar_SERVO
{
public:
  Avatar_SERVO();
  void uartSentData(int arr[], int size);               // 发送信息
  void uartSentData1(int arr[], int size); 
  int readServoPosition(int servoNum);                  // 获得舵机位置信息
  void setServoNum(int servoNum);                       // 设置舵机号
  void servoMove(int servoNum, float angle, int time); // 舵机号、位置、时间
  void setDeviation(int servoNum, int deviation);       // 设置舵机偏移
  int readDeviation(int servoNum);                      // 读取舵机偏移
  void torqueOff(int servoNum);                         // 取消扭矩
  void torqueOn(int servoNum);                          // 开启扭矩
private:
  int verify = 0;
  int ServoPosition = 0;
};
#endif
