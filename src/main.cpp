#include "LegControl_task.h"
#include "gait_prg.h"
#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"
using namespace std;
#include "avatar_servo.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Freenove_WS2812_Lib_for_ESP32.h"
#include "ESP32Servo.h"
#define LEDS_COUNT 2
#define LEDS_PIN 23
#define CHANNEL 0
Servo servo1;
Servo servo2;
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

// 全局变量
uint32_t LegControl_round; // 机器人回合数
Hexapod hexapod;		   // 机器人结构体
float LXValue;
float RYValue;
float RXValue;
float CLXValue;
float CRXValue;
float CRYValue;
Gait_prg gait_prg;	  // 步态规划
uint32_t round_time;  // 回合时间
Thetas leg_offset[6]; // 腿部关节角偏移，用于将舵机相对机器人本体的角度换算至相对舵机本身的角度

Avatar_SERVO arm;
// 函数
static uint32_t code_time_start, code_time_end, code_time; // 用于计算程序运行时间，保证程序隔一段时间跑一遍
static void remote_deal(void);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

typedef struct
{
	unsigned char index;
	unsigned char up;
	unsigned char down;
	void (*operation)(void);
} KEY_TABLE;

unsigned char funIndex = 0;
void (*current)(void);
void menu11(void);
void menu12(void);
void menu13(void);

// 定义按键操作数据
KEY_TABLE table[3] = {
	{0, 2, 1, (*menu11)},
	{1, 0, 2, (*menu12)},
	{2, 1, 0, (*menu13)},
};
void menu11(void)
{
	oled.clearDisplay();  // 清屏
	oled.setTextSize(1);  // 设置字体大小
	oled.setCursor(0, 0); // 设置显示位置
	oled.println("->MOVE MODE");
	oled.setCursor(0, 10); // 设置显示位置
	oled.println("POSITION MODE");
	oled.setCursor(0, 20); // 设置显示位置
	oled.println("ANGEL MODE");
	oled.display(); // 开显示
}

void menu12(void)
{
	oled.clearDisplay();  // 清屏
	oled.setTextSize(1);  // 设置字体大小
	oled.setCursor(0, 0); // 设置显示位置
	oled.println("MOVE MODE");
	oled.setCursor(0, 10); // 设置显示位置
	oled.println("->POSITION MODE");
	oled.setCursor(0, 20); // 设置显示位置
	oled.println("ANGEL MODE");
	oled.display(); // 开显示
}

void menu13(void)
{
	oled.clearDisplay();  // 清屏
	oled.setTextSize(1);  // 设置字体大小
	oled.setCursor(0, 0); // 设置显示位置
	oled.println("MOVE MODE");
	oled.setCursor(0, 10); // 设置显示位置
	oled.println("POSITION MODE");
	oled.setCursor(0, 20); // 设置显示位置
	oled.println("->ANGEL MODE");
	oled.display(); // 开显示
}
void Hexapod::modeSelect()
{

	while (1)
	{
		if (PS4.Up())
		{
			delay(80);
			if (PS4.Up())
			{
				funIndex = table[funIndex].up;
				current = table[funIndex].operation; // 根据需要获取对应需要执行的函数
				(*current)();
			}
			// 执行获取到的函数
		}

		if (PS4.Down())
		{
			delay(80);
			if (PS4.Down())
			{
				funIndex = table[funIndex].down;
				current = table[funIndex].operation; // 根据需要获取对应需要执行的函数
				(*current)();						 // 执行获取到的函数
			}
		}

		if (PS4.Square())
		{
			switch (funIndex) // 获取按键对应序号
			{
			case 0:
				this->mode = HEXAPOD_MOVE;
				break;
			case 1:
				this->mode = HEXAPOD_BODY_POS_CONTROL;
				break;
			case 2:
				this->mode = HEXAPOD_BODY_ANGEL_CONTROL;
				break;
			}
			break;
		}
	}
}
void motor_init()
{
	pinMode(rightmotor1, OUTPUT);
	pinMode(rightmotor2, OUTPUT);
	pinMode(leftmotor1, OUTPUT);
	pinMode(leftmotor2, OUTPUT);

	ledcSetup(rightmotord1, 5000, 10);
	ledcAttachPin(rightmotor1, rightmotord1);
	ledcSetup(rightmotord2, 5000, 10);
	ledcAttachPin(rightmotor2, rightmotord2);

	ledcSetup(leftmotord1, 5000, 10);
	ledcAttachPin(leftmotor1, leftmotord1);
	ledcSetup(leftmotord2, 5000, 10);
	ledcAttachPin(leftmotor2, leftmotord2);
}
void speed_control(int left, int right)
{
	if ((left < 0) && (right < 0))
	{
		ledcWrite(leftmotord1, 0);
		ledcWrite(leftmotord2, -left);
		ledcWrite(rightmotord1, 0);
		ledcWrite(rightmotord2, -right);
	}
	else if ((left < 0))
	{
		ledcWrite(leftmotord1, 0);
		ledcWrite(leftmotord2, -left);
		ledcWrite(rightmotord1, right);
		ledcWrite(rightmotord2, 0);
	}
	else if (right < 0)
	{
		ledcWrite(leftmotord1, left);
		ledcWrite(leftmotord2, 0);
		ledcWrite(rightmotord1, 0);
		ledcWrite(rightmotord2, -right);
	}
	else
	{
		ledcWrite(leftmotord1, left);
		ledcWrite(leftmotord2, 0);
		ledcWrite(rightmotord1, right);
		ledcWrite(rightmotord2, 0);
	}
}

void setup()
{
	Serial.begin(115200);
	Serial2.begin(115200, SERIAL_8N1, 18, 5);
	Serial1.begin(115200, SERIAL_8N1, 16, 4);
	servo1.attach(14);
	servo2.attach(15);
	hexapod.Init();
	gait_prg.Init();
	delay(100);
	motor_init(); // 电机初始化
	PS4.begin("ba:ba:ca:ea:02:01");
	uint8_t pairedDeviceBtAddr[20][6];
	int count = esp_bt_gap_get_bond_device_num();
	esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
	for (int i = 0; i < count; i++)
	{
		esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
	}
	// 手柄初始化
	if (!oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
	{
		Serial.println(F("SSD1306 allocation failed"));
	}
	oled.clearDisplay();
	oled.display();
	oled.setTextSize(1);
	oled.setTextColor(SSD1306_WHITE);
	// OLED屏初始化
	menu11();
	hexapod.mode_select(); // 开机默认行走模式
	strip.begin();
	strip.setBrightness(10);
	arm.servoMove(20, 150, 1000);
}
void loop()
{
	
	code_time_start = millis(); // 获取当前时间
	remote_deal();
	if (hexapod.velocity.omega >= 0)
		LegControl_round = (++LegControl_round) % N_POINTS; // 控制回合自增长
	else
	{
		if (LegControl_round == 0)
			LegControl_round = N_POINTS - 1;
		else
			LegControl_round--;
	}
	/*步态控制*/
	gait_prg.CEN_and_pace_cal();
	gait_prg.gait_proggraming();
	/*开始移动*/
	round_time = gait_prg.get_pace_time() / N_POINTS;
	hexapod.move(round_time);
	// 计算程序运行时间
	code_time_end = millis();					 // 获取当前时间
	code_time = code_time_end - code_time_start; // 做差获取程序运行时间（8ms）
	if (code_time < round_time)
		delay(round_time - code_time); // 保证程序执行周期等于回合时间
	else
		delay(1); // 至少延时1ms
	delay(10);
}

// 初始化腿部变量
void Hexapod::Init(void)
{
	for (int i = 0; i < 6; i++) // 复位
	{
		arm.servoMove(3 * i + 1, 150, 2000); // 设置机械腿角度1
		arm.servoMove(3 * i + 2, 130, 2000); // 设置机械腿角度2
		arm.servoMove(3 * i + 3, 140, 2000); // 设置机械腿角度3
	}
	delay(2000);
	leg_offset[0] = Thetas(PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[1] = Thetas(0.0f, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[2] = Thetas(-PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[3] = Thetas(3 * PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[4] = Thetas(PI, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[5] = Thetas(-3 * PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	mpu_pid_x.Init(MPU_X_PID_KP, MPU_X_PID_KI, MPU_X_PID_KD, CIR_OFF);
	mpu_pid_y.Init(MPU_Y_PID_KP, MPU_Y_PID_KI, MPU_Y_PID_KD, CIR_OFF);
	mpu_pid_z.Init(MPU_Y_PID_KP, MPU_Y_PID_KI, MPU_Y_PID_KD, CIR_OFF);
	velocity_fof[0].set_k_filter(VELOCITY_FOF_K);
	velocity_fof[1].set_k_filter(VELOCITY_FOF_K);
	velocity_fof[2].set_k_filter(VELOCITY_FOF_K);
	body_pos_fof[0].set_k_filter(BODY_POS_FOF_K);
	body_pos_fof[1].set_k_filter(BODY_POS_FOF_K);
	body_pos_fof[2].set_k_filter(BODY_POS_FOF_K);
	body_angle_fof[0].set_k_filter(BODY_ANGLE_FOF_K);
	body_angle_fof[1].set_k_filter(BODY_ANGLE_FOF_K);
	body_angle_fof[2].set_k_filter(BODY_ANGLE_FOF_K);
}

// 计算机器人速度
void Hexapod::velocity_cal()
{
	if (this->mode != HEXAPOD_MOVE) // 若不是行走模式则速度为0
	{
		velocity.Vx = 0;
		velocity.Vy = 0;
		velocity.omega = 0;
	}
	else
	{
		RXValue =0.8* PS4.RStickX();
		RYValue =0.8*  PS4.RStickY();
		LXValue =0.8*  PS4.LStickX();
		CRXValue = RXValue;
		CRYValue = RYValue;
		CLXValue = LXValue;

		// 使用std::abs计算绝对值
		RXValue = std::abs(RXValue); // 计算RXValue的绝对值
		RYValue = std::abs(RYValue); // 计算RYValue的绝对值
		LXValue = std::abs(LXValue); // 计算LXValue的绝对值

		// 直接计算并检查接近零的值
		velocity.Vx = (std::abs(RXValue) < 0.0001f) ? 0 : 0.4f * RXValue;	 // 如果RXValue的绝对值小于0.0001，则Vx为0，否则为0.4f * RXValue
		velocity.Vy = (std::abs(RYValue) < 0.0001f) ? 0 : 0.4f * RYValue;	 // 如果RYValue的绝对值小于0.0001，则Vy为0，否则为0.4f * RYValue
		velocity.omega = (std::abs(LXValue) < 0.0001f) ? 0 : 0.4f * LXValue; // 如果LXValue的绝对值小于0.0001，则omega为0，否则为0.4f * LXValue

		gait_prg.set_velocity(velocity); // 设置步态程序的速度
	}
}

void Hexapod::body_position_cal()
{
	if (this->mode != HEXAPOD_BODY_ANGEL_CONTROL) // 除了姿态控制模式，其他情况下都能控制z轴高度
		body_pos.z += ROTATE_BODY_POS_SENSI * PS4.LStickY();
	if (this->mode == HEXAPOD_BODY_POS_CONTROL) // 若是身体位置控制模式则计算xy位置
	{
		// body_pos.y += ROTATE_BODY_POS_SENSI * remote_data.right_VETC;
		// body_pos.x += ROTATE_BODY_POS_SENSI * remote_data.right_HRZC;
		body_pos.y = HEXAPOD_MAX_Y / 660.0f * PS4.RStickY();
		body_pos.x = -HEXAPOD_MIN_X / 660.0f * PS4.RStickX();
	}
	// 限制数值
	value_limit(body_pos.z, HEXAPOD_MIN_HEIGHT, HEXAPOD_MAX_HEIGHT);
	value_limit(body_pos.y, HEXAPOD_MIN_Y, HEXAPOD_MAX_Y);
	value_limit(body_pos.x, HEXAPOD_MIN_X, HEXAPOD_MAX_X);
	// 一阶低通滤波
	// body_pos.y = body_angle_fof[1].cal(5.f * body_pos.y);
	// body_pos.x = body_angle_fof[2].cal(5.f * body_pos.x);
	body_pos.y = 5.f * body_pos.y;
	body_pos.x = 5.f * body_pos.x;
	gait_prg.set_body_position(body_pos);
}

void Hexapod::body_angle_cal()
{
	// mpu_angle = mpu6050.get_angle();			  // 获取陀螺仪角度
	if (this->mode != HEXAPOD_BODY_ANGEL_CONTROL) // 若不是姿态控制模式则直接返回
		return;
	if (this->mpu_flag == false && this->mpu_sw == MPU_ON) // 第一次进入陀螺仪模式
	{
		this->mpu_angle_set = this->mpu_angle; // 将设定角度等于陀螺仪角度
		this->mpu_flag = true;				   // 标志位置1
	}
	if (this->mpu_sw == MPU_OFF) // 不是陀螺仪模式则标志位置零
		this->mpu_flag = false;
	if (this->mpu_sw == MPU_ON)
	{
		// mpu_angle_set.y- = ROTATE_BODY_ANGLE_SENSI * PS4.LStickX();
		// mpu_angle_set.x== ROTATE_BODY_ANGLE_SENSI * PS4.LStickY();
		// mpu_angle_set.z- = ROTATE_BODY_ANGLE_SENSI * PS4.RStickX();
		// //  body_angle.x = this->mpu_angle.x;
		// //  body_angle.y = this->mpu_angle.y;	QWE
		// //  float mpu_x_add = mpu_angle_set.x - mpu_angle.x;
		// //  float mpu_y_add = mpu_angle_set.y - mpu_angle.y;
		// body_angle.x -= this->mpu_pid_x.cal(mpu_angle.x, mpu_angle_set.x);
		// body_angle.y -=this->mpu_pid_y.cal(mpu_angle.y, mpu_angle_set.y);
		// body_angle.z -= this->mpu_pid_z.cal(mpu_angle.z, mpu_angle_set.z);
		// // body_angle.x = this->mpu_angle.x + this->mpu_pid_x.cal(mpu_angle.x, mpu_angle_set.x);
		// // body_angle.y = this->mpu_angle.y - this->mpu_pid_y.cal(mpu_angle.y, mpu_angle_set.y);
		// // body_angle.z += (mpu_angle_set.z - mpu_angle.z); //陀螺仪z轴零漂严重，暂时不使用
		body_angle.y = -ROTATE_BODY_ANGLE_SENSI * PS4.LStickX();
		body_angle.x = ROTATE_BODY_ANGLE_SENSI * PS4.LStickY();
		body_angle.z = ROTATE_BODY_ANGLE_SENSI * PS4.RStickX(); // 映射手柄绝对值
	}
	else
	{
		body_angle.x = body_angle_fof[0].cal(-0.02f * PS4.LStickX());
		body_angle.y = body_angle_fof[1].cal(-0.02f * PS4.LStickY());
		body_angle.z = body_angle_fof[2].cal(-0.02f * PS4.RStickX());
	}
	value_limit(body_angle.x, HEXAPOD_MIN_X_ROTATE, HEXAPOD_MAX_X_ROTATE);
	value_limit(body_angle.y, HEXAPOD_MIN_Y_ROTATE, HEXAPOD_MAX_Y_ROTATE);
	value_limit(body_angle.z, HEXAPOD_MIN_Z_ROTATE, HEXAPOD_MAX_Z_ROTATE);

	gait_prg.set_body_rotate_angle(body_angle);
}
// 计算机器人速度
void Hexapod::velocity_cal(int RXValue, int RYValue, int LXValue)
{
	// 反转逻辑
	CRXValue = RXValue; // 如果RXValue小于0，则CRXValue等于RXValue，否则等于-RXValue
	CRYValue = RYValue; // 如果RYValue小于0，则CRYValue等于RYValue，否则等于-RYValue
	CLXValue = LXValue; // 如果LXValue小于0，则CLXValue等于LXValue，否则等于-LXValue

	// 使用std::abs计算绝对值
	RXValue = std::abs(RXValue); // 计算RXValue的绝对值
	RYValue = std::abs(RYValue); // 计算RYValue的绝对值
	LXValue = std::abs(LXValue); // 计算LXValue的绝对值

	// 直接计算并检查接近零的值
	velocity.Vx = (std::abs(RXValue) < 0.0001f) ? 0 : 0.4f * RXValue;	 // 如果RXValue的绝对值小于0.0001，则Vx为0，否则为0.4f * RXValue
	velocity.Vy = (std::abs(RYValue) < 0.0001f) ? 0 : 0.4f * RYValue;	 // 如果RYValue的绝对值小于0.0001，则Vy为0，否则为0.4f * RYValue
	velocity.omega = (std::abs(LXValue) < 0.0001f) ? 0 : 0.4f * LXValue; // 如果LXValue的绝对值小于0.0001，则omega为0，否则为0.4f * LXValue

	gait_prg.set_velocity(velocity); // 设置步态程序的速度
}

void Hexapod::body_position_cal(int xValue, int yValue, int zValue)
{
	if (this->mode != HEXAPOD_BODY_ANGEL_CONTROL) // 除了姿态控制模式，其他情况下都能控制z轴高度
		body_pos.z = ROTATE_BODY_POS_SENSI * zValue;
	if (this->mode == HEXAPOD_BODY_POS_CONTROL) // 若是身体位置控制模式则计算xy位置
	{

		body_pos.y = HEXAPOD_MAX_Y / 660.0f * yValue;
		body_pos.x = -HEXAPOD_MIN_X / 660.0f * xValue;
	}
	// 限制数值
	value_limit(body_pos.z, HEXAPOD_MIN_HEIGHT, HEXAPOD_MAX_HEIGHT);
	value_limit(body_pos.y, HEXAPOD_MIN_Y, HEXAPOD_MAX_Y);
	value_limit(body_pos.x, HEXAPOD_MIN_X, HEXAPOD_MAX_X);
	// 一阶低通滤波

	body_pos.y = 5.f * body_pos.y;
	body_pos.x = 5.f * body_pos.x;
	gait_prg.set_body_position(body_pos);
}

void Hexapod::body_angle_cal(int xValue, int yValue, int zValue)
{
	// mpu_angle = mpu6050.get_angle();			  // 获取陀螺仪角度
	if (this->mode != HEXAPOD_BODY_ANGEL_CONTROL) // 若不是姿态控制模式则直接返回
		return;
	if (this->mpu_flag == false && this->mpu_sw == MPU_ON) // 第一次进入陀螺仪模式
	{
		this->mpu_angle_set = this->mpu_angle; // 将设定角度等于陀螺仪角度
		this->mpu_flag = true;				   // 标志位置1
	}
	if (this->mpu_sw == MPU_OFF) // 不是陀螺仪模式则标志位置零
		this->mpu_flag = false;
	if (this->mpu_sw == MPU_ON)
	{
		body_angle.y = yValue;
		body_angle.x = -xValue;
		body_angle.z = zValue; // 映射手柄绝对值
	}
	else
	{
		// body_angle.x = body_angle_fof[0].cal(-0.02f * PS4.LStickX());
		// body_angle.y = body_angle_fof[1].cal(-0.02f * PS4.LStickY());
		// body_angle.z = body_angle_fof[2].cal(-0.02f * PS4.RStickX());
	}
	value_limit(body_angle.x, HEXAPOD_MIN_X_ROTATE, HEXAPOD_MAX_X_ROTATE);
	value_limit(body_angle.y, HEXAPOD_MIN_Y_ROTATE, HEXAPOD_MAX_Y_ROTATE);
	value_limit(body_angle.z, HEXAPOD_MIN_Z_ROTATE, HEXAPOD_MAX_Z_ROTATE);
	gait_prg.set_body_rotate_angle(body_angle);
}

/*
 *@brief 检查拨轮数据，符合要求就归零
 *@param remote_data 遥控数据
 */
void Hexapod::body_angle_and_pos_zero()
{
	this->body_angle.zero();
	this->body_pos.zero();
	this->mpu_angle_set.zero();
}

void Hexapod::mode_select()
{
	// switch (remote_data.S1)
	// {
	// case 1:					 // 拨杆在上面
	// 	mode = HEXAPOD_MOVE; // 移动模式
	// 	break;
	// case 2:								   // 拨杆在下面
	// 	mode = HEXAPOD_BODY_ANGEL_CONTROL; // 机身旋转角度控制
	// 	break;
	// case 3:								 // 拨杆在中间
	// 	mode = HEXAPOD_BODY_POS_CONTROL; // 机身位置控制
	// default:
	// 	break;
	// }
	// switch (remote_data.S2)
	// {
	// case 0:
	// 	mpu_sw = MPU_ON;
	// 	break;
	// case 1:
	// 	mpu_sw = MPU_OFF;
	// 	break;
	// default:
	// 	break;
	// }
	mode = HEXAPOD_MOVE;
}

static void remote_deal(void)
{
	// static RC_remote_data_t remote_data;
	// remote_data = Remote_read_data();
	// hexapod.mode_select();
	static uint16_t armPos = 150;
	if (PS4.Options())
	{
		hexapod.modeSelect();
		hexapod.body_angle_and_pos_zero();
	}
	else if (PS4.Share())
	{
		hexapod.body_angle_and_pos_zero();
	}

	else if (PS4.Up())
	{
		delay(2);
		if (PS4.Up())
		{
			arm.servoMove(20, armPos--, 2);
		}
	}
	else if (PS4.Down())
	{
		delay(2);
		if (PS4.Down())
		{
			arm.servoMove(20, armPos++, 2);
		}
	}
	else if (PS4.Circle())
	{
		strip.setLedColorData(0, 255, 0, 0);
		strip.setLedColorData(1, 255, 0, 0);
		strip.show();
	}
	else if (PS4.Triangle())
	{
		strip.setLedColorData(0, 0, 255, 0);
		strip.setLedColorData(1, 0, 255, 0);
		strip.show();
	}
	else if (PS4.Cross())
	{
		strip.setLedColorData(0, 0, 0, 255);
		strip.setLedColorData(1, 0, 0, 255);
		strip.show();
	}
	else if (PS4.L1())
	{
		speed_control(1000, 1000);
	}
	else if (PS4.L2())
	{
		speed_control(0, 0);
	}
	else if (PS4.Left())
	{
		servo2.write(0);
		servo1.write(180);
		delay(1000);

	}
	else if (PS4.Right())
	{
		servo1.write(0);
		servo2.write(180);
		delay(1000);
	}
	hexapod.velocity_cal();
	hexapod.body_angle_cal();
	hexapod.body_position_cal();
}

/*
 * @brief 让机器人动起来
 * @param round_time 回合时间，单位ms
 */
void Hexapod::move(uint32_t round_time)
{

	Thetas theta_temp;
	for (int i = 0; i < 6; i++) // 遍历六条腿
	{
		theta_temp = (gait_prg.actions[i].thetas[LegControl_round]) - leg_offset[i]; // 计算当前回合的关节角度
		if (theta_temp.angle[0] <= -2.0f / 3.0f * PI)								 // 如果第一个关节的角度小于-120度
		{
			theta_temp.angle[0] += 2 * PI; // 调整角度到0-360度范围内
		}
		int servo_base_index = 0;														// 舵机基础索引
		bool reverse_angle_0 = false, reverse_angle_1 = false, reverse_angle_2 = false; // 关节角度反转标志
		// 根据CRXValue和CRYValue的值调整舵机基础索引和关节角度反转标志
		if ((CRXValue < 0 && CRYValue < 0) || CLXValue < 0) // 如果xy都小于零或角速度小于零，绕中点旋转
		{
			servo_base_index = 16 - 3 * i;
			reverse_angle_0 = true;
			reverse_angle_1 = true;
		}
		else if ((CRXValue >= 0 && CRYValue >= 0) && CLXValue == 0) // 如果xy都大于零
		{
			servo_base_index = 3 * i + 1;
			reverse_angle_0 = true;
			reverse_angle_1 = true;
		}
		else if (CRXValue < 0 && CRYValue >= 0) // 如果x小于零，y大于零，绕y轴旋转
		{
			servo_base_index = i < 3 ? 3 * i + 10 : 3 * i - 8;
			reverse_angle_1 = true;
		}
		else if (CRXValue >= 0 && CRYValue < 0) // 如果x大于零，y小于零，绕x轴旋转
		{
			servo_base_index = i < 3 ? 7 - 3 * i : 25 - 3 * i;
			reverse_angle_1 = true;
		}
		else if (CLXValue > 0 && (CRXValue == 0 && CRYValue == 0)) // 如果角速度大于零
		{
			servo_base_index = 3 * i + 1;
			reverse_angle_1 = true;
		}
		// 根据计算出的舵机基础索引和关节角度反转标志，控制舵机运动
		arm.servoMove(servo_base_index, (reverse_angle_0 ? -1 : 1) * theta_temp.angle[0] / PI * 180 + 150, round_time);
		// Serial.print(servo_base_index);
		// Serial.print(":");
		// Serial.println((reverse_angle_0 ? -1 : 1) * theta_temp.angle[0] / PI * 180 + 150);
		arm.servoMove(servo_base_index + 1, (reverse_angle_1 ? -1 : 1) * theta_temp.angle[1] / PI * 180 + 150, round_time);
		// Serial.print(servo_base_index + 1);
		// Serial.print(":");
		// Serial.println((reverse_angle_1 ? -1 : 1) * theta_temp.angle[1] / PI * 180 + 150);
		arm.servoMove(servo_base_index + 2, (reverse_angle_2 ? -1 : 1) * theta_temp.angle[2] / PI * 180 + 250, round_time);
		// Serial.print(servo_base_index + 2);
		// Serial.print(":");
		// Serial.println((reverse_angle_2 ? -1 : 1) * theta_temp.angle[2] / PI * 180 + 150);
	}
}
