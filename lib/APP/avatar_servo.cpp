#include <avatar_servo.h>
Avatar_SERVO::Avatar_SERVO()
{
}
void Avatar_SERVO::uartSentData(int arr[], int size)
{
  for (int i = 0; i < size; i++)
  {
    Serial2.write(arr[i]);
  }
  int a = 0;
  for (int j = 2; j < size; j++)
  {
    a += arr[j];
  }
  verify = (~a) & 0xFF;
  Serial2.write(verify);
}
void Avatar_SERVO::uartSentData1(int arr[], int size)
{
  for (int i = 0; i < size; i++)
  {
    Serial1.write(arr[i]);
  }
  int a = 0;
  for (int j = 2; j < size; j++)
  {
    a += arr[j];
  }
  verify = (~a) & 0xFF;
  Serial1.write(verify);
}
int Avatar_SERVO::readServoPosition(int servoNum)
{
  int pos[7];
  pos[0] = 0x12;
  pos[1] = 0x4C;
  pos[2] = servoNum;
  pos[3] = 0x04;
  pos[4] = 0x02;
  pos[5] = 0x38;
  pos[6] = 0x02;
  uartSentData(pos, 7); // 发送位置读取命令

  delay(5);
  if (Serial2.available() > 0)
  {
    String incomingString = Serial2.readStringUntil('\n'); // 读取一整行字符串，直到遇到换行符
    String highPosition = incomingString.substring(5, 6);
    String lowPosition = incomingString.substring(6, 7); // 截取返回的位置信息

    int a = highPosition[0];
    int b = lowPosition[0];
    float pos = a * 256 + b;
    pos = (pos / 0XFFF) * 300;
    return pos;
  }
  else
    return 0;
}

void Avatar_SERVO::setServoNum(int servoNum)
{
  int num[7];
  num[0] = 0x12;
  num[1] = 0x4C;
  num[2] = 0xFE;
  num[3] = 0x04;
  num[4] = 0x03;
  num[5] = 0x05;
  num[6] = servoNum;
  uartSentData(num, 7);
}

void Avatar_SERVO::servoMove(int servoNum, float angle, int time) // 舵机号、位置、时间
{
  angle = constrain(angle, 0, 300);
  // value_limit(angle,0,300);
  int position = map(angle, 0, 300, 0, 4095);
  int move[12];
  move[0] = 0x12;
  move[1] = 0x4C;
  move[2] = 0xFE;
  move[3] = 0x09;
  move[4] = 0x83;
  move[5] = 0x2A;
  move[6] = 0x04;
  move[7] = servoNum;
  move[8] = GET_HIGH_BYTE(position);
  move[9] = GET_LOW_BYTE(position);
  move[10] = GET_HIGH_BYTE(time);
  move[11] = GET_LOW_BYTE(time);
  if (servoNum < 10)
  {
    uartSentData1(move, 12);
  }
  else
    uartSentData(move, 12);
}

void Avatar_SERVO::setDeviation(int servoNum, int angle)
{
  int deviation = map(angle, -300, 300, -4095, 4095);
  int dev[8];
  dev[0] = 0x12;
  dev[1] = 0x4C;
  dev[2] = servoNum;
  dev[3] = 0x05; // 长度
  dev[4] = 0x03; // 指令类型
  dev[5] = 0x14;
  dev[6] = GET_HIGH_BYTE(deviation);
  dev[7] = GET_LOW_BYTE(deviation); // 参数
  uartSentData(dev, 8);             // 计算校验位/发送
}

int Avatar_SERVO::readDeviation(int servoNum)
{
  int dev[7];
  dev[0] = 0x12;
  dev[1] = 0x4C;
  dev[2] = servoNum;
  dev[3] = 0x04; // 长度（参数长度+2）
  dev[4] = 0x02; // 指令类型
  dev[5] = 0x14;
  dev[6] = 0x02;        // 要读取的参数长度
  uartSentData(dev, 7); // 计算校验位/发送
  delay(5);
  if (Serial2.available() > 0)
  {
    String incomingString = Serial2.readStringUntil('\n'); // 读取一整行字符串，直到遇到换行符
    String highByte = incomingString.substring(5, 6);
    String lowByte = incomingString.substring(6, 7); // 截取返回的位置信息

    int a = highByte[0];
    int b = lowByte[0];
    float dev = a * 256 + b;
    if (dev < 0xFFF)
    {
      dev = (dev / 0XFFF) * 300;
    }
    else
    {
      dev = dev - 0x10000;
    }
    dev = (dev / 0XFFF) * 300;
    return dev;
  }
  else
    return 888;
}
void Avatar_SERVO::torqueOff(int servoNum) // 取消扭矩
{
  int buf[7];
  buf[0] = 0x12;
  buf[1] = 0x4C;
  buf[2] = servoNum; // id
  buf[3] = 0x04;     // 长度
  buf[4] = 0x03;
  buf[5] = 0x28;
  buf[6] = 0x00;
  uartSentData(buf, 7);
}
void Avatar_SERVO::torqueOn(int servoNum) // 取消扭矩
{
  int buf[7];
  buf[0] = 0x12;
  buf[1] = 0x4C;
  buf[2] = servoNum; // id
  buf[3] = 0x04;     // 长度
  buf[4] = 0x03;
  buf[5] = 0x28;
  buf[6] = 0x01;
  uartSentData(buf, 7);
}