#ifndef LEGCONTROL_TASK_H
#define LEGCONTROL_TASK_H

#include "my_math.h"
#include "gait_prg.h"
#include "SCServo.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#define OUTPUT_READABLE_YAWPITCHROLL

#define LEG_JOINT2_OFFSET 0 // PI / 2
#define LEG_JOINT3_OFFSET 0 //-2 * PI / 9

#define HEXAPOD_MIN_HEIGHT -70.0f
#define HEXAPOD_MAX_HEIGHT 70.0f
#define HEXAPOD_MIN_X -6.5f
#define HEXAPOD_MAX_X 6.5f
#define HEXAPOD_MIN_Y -6.5f
#define HEXAPOD_MAX_Y 6.5f

#define HEXAPOD_MIN_X_ROTATE -15.0f / 180 * PI // 绕X轴旋转角度最小为-15度
#define HEXAPOD_MAX_X_ROTATE 15.0f / 180 * PI  // 绕X轴旋转角度最大为 15度
#define HEXAPOD_MIN_Y_ROTATE -15.0f / 180 * PI // 绕y轴旋转角度最小为-10度
#define HEXAPOD_MAX_Y_ROTATE 15.0f / 180 * PI  // 绕y轴旋转角度最大为 10度
#define HEXAPOD_MIN_Z_ROTATE -25.0f / 180 * PI // 绕z轴旋转角度最小为-25度
#define HEXAPOD_MAX_Z_ROTATE 25.0f / 180 * PI  // 绕z轴旋转角度最大为 25度

/*PID*/
#define MPU_X_PID_KP 0.015f
#define MPU_X_PID_KI 0.0f
#define MPU_X_PID_KD 0.5f

#define MPU_Y_PID_KP 0.015f
#define MPU_Y_PID_KI 0.0f
#define MPU_Y_PID_KD 0.5f

/*FOF一阶低通滤波参数*/
#define VELOCITY_FOF_K 0.5f
#define BODY_POS_FOF_K 0.5f
#define BODY_ANGLE_FOF_K 0.5f

#define ROTATE_BODY_ANGLE_SENSI 0.002f // 控制角度灵敏度
#define ROTATE_BODY_POS_SENSI 0.002f   // 控制位置灵敏度
float leftDev = 1, rightDev = 1;	// 无陀螺仪用它
float mLeftDev = 1, mRightDev = 1;	// 有陀螺仪用它
uint8_t devStatus = 1;      // MPU6050操作后的返回状态（0 = 成功，!0 = 错误）
Thetas leg_offset[6]; // 腿部关节角偏移，用于将舵机相对机器人本体的角度换算至相对舵机本身的角度
float CLXValue, CRXValue, CRYValue;
uint32_t LegControl_round; // 机器人回合数

typedef enum
{
	HEXAPOD_MOVE,
	HEXAPOD_BODY_ANGEL_CONTROL,
	HEXAPOD_BODY_POS_CONTROL,
} Hexapod_mode_e;

typedef enum
{
	MPU_ON,
	MPU_OFF,
} MPU_SW_e;

class Hexapod
{
public:
	SCSCL &arm;	  // 舵机控制器
	Gait_prg &gait_prg;	  // 步态规划
	Velocity velocity;	  // 机器人速度
	Hexapod_mode_e mode;  // 机器人模式
	MPU_SW_e mpu_sw;	  // 是否由陀螺仪控制
	Position3 body_pos;	  // 机体位置
	Position3 body_angle; // 机体角度
	Position3 mpu_angle;
	Position3 mpu_angle_set;
	PID mpu_pid_x; // x轴pid
	PID mpu_pid_y; // y轴pid
	PID mpu_pid_z; // y轴pid
	First_order_filter velocity_fof[3];
	First_order_filter body_pos_fof[3];
	First_order_filter body_angle_fof[3];
	Hexapod(SCSCL &arm, Gait_prg &gait_prg) : arm(arm), gait_prg(gait_prg) {} // 构造函数，接受一个Avatar_SERVO对象和一个Gait_prg对象的引用
	bool mpu_flag;
	void Init(uint8_t mode);
	void velocity_cal(int RXValue, int RYValue, int LXValue);
	void body_position_cal(int xValue, int yValue, int zValue); // 控制机体位置平移(x身体左右平移)，(y身体前后平移), (z表示上下平移)
	void body_angle_cal(int xValue, int yValue, int zValue);	// 控制机体角度(平转) x(左，右平转), y(头高和低), z(身体左右倾斜) 参数由对象body_angle传递
	void mode_select(uint8_t Mode);
	void body_angle_and_pos_zero();
	void move(uint32_t round_time);
	void remote_deal();
	void modeSelect();
	void Crab_walking(uint16_t timeDelay);
};

class UserMpu
{
	public:
	MPU6050 mpu;
	// MPU 控制/状态变量
	bool dmpReady = false;  // 如果 DMP 初始化成功，则设置为 true
	uint8_t mpuIntStatus;   // 保存 MPU 的实际中断状态字节
	uint16_t packetSize;    // 预期的 DMP 数据包大小（默认为 42 字节）
	uint16_t fifoCount;     // FIFO 中当前的所有字节计数
	uint8_t fifoBuffer[64]; // FIFO 存储缓冲区
	// 方向/运动变量
	Quaternion q;           // [w, x, y, z]         四元数容器
	VectorInt16 aa;         // [x, y, z]            加速度传感器测量值
	VectorInt16 gy;         // [x, y, z]            陀螺仪传感器测量值
	VectorInt16 aaReal;     // [x, y, z]            无重力加速度传感器测量值
	VectorInt16 aaWorld;    // [x, y, z]            世界坐标系下的加速度传感器测量值
	VectorFloat gravity;    // [x, y, z]            重力向量
	float euler[3];         // [psi, theta, phi]    欧拉角容器
	float ypr[3];           // [yaw, pitch, roll]   偏航/俯仰/翻滚容器和重力向量（弧度制）
	uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};
	void mpu_Init();
	void mpu_cab();
	float mpu_Get_angle(char x);
};

// 初始化腿部变量
void Hexapod::Init(uint8_t mode)
{
	static int flag = 0;
	// 初始化腿部偏移角度
	if (mode == 1)
	{
		leg_offset[0] = Thetas(PI / 2.0, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[1] = Thetas(0.0f, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[2] = Thetas(-PI / 2.0, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[3] = Thetas(PI / 2, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[4] = Thetas(PI, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[5] = Thetas(-PI / 2, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		arm.servoMove(2, 110, 500);
		arm.servoMove(17, 110, 500);
		delay(500);
		arm.servoMove(1, 195, 500);
		arm.servoMove(16, 195, 500);
		delay(500);
		arm.servoMove(2, 130, 500);
		arm.servoMove(17, 130, 500);
		delay(500);
		arm.servoMove(8, 110, 500);
		arm.servoMove(11, 110, 500);
		delay(500);
		arm.servoMove(7, 105, 500);
		arm.servoMove(10, 105, 500);
		delay(500);
		arm.servoMove(8, 130, 500);
		arm.servoMove(11, 130, 500);
		delay(500);
		flag = 1;
	}
	if (flag == 0 && mode == 0)
	{
		leg_offset[0] = Thetas(PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[1] = Thetas(0.0f, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[2] = Thetas(-PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[3] = Thetas(3 * PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[4] = Thetas(PI, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[5] = Thetas(-3 * PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		for (int i = 0; i < 6; i++) // 遍历六条腿进行初始化
		{
			arm.servoMove(3 * i + 1, 150, 1000); // 初始化第一关节角度为150度
			arm.servoMove(3 * i + 2, 105, 1000); // 初始化第二关节角度为130度
			arm.servoMove(3 * i + 3, 105, 1000); // 初始化第三关节角度为140度
		}

		delay(1000); // 延时等待舵机移动完成
	}
	if (flag == 1 && mode == 0)
	{
		leg_offset[0] = Thetas(PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[1] = Thetas(0.0f, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[2] = Thetas(-PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[3] = Thetas(3 * PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[4] = Thetas(PI, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		leg_offset[5] = Thetas(-3 * PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
		arm.servoMove(2, 110, 500);
		arm.servoMove(17, 110, 500);
		delay(500);
		arm.servoMove(1, 150, 500);
		arm.servoMove(16, 150, 500);
		delay(500);
		arm.servoMove(2, 130, 500);
		arm.servoMove(17, 130, 500);
		delay(500);
		arm.servoMove(8, 110, 500);
		arm.servoMove(11, 110, 500);
		delay(500);
		arm.servoMove(7, 150, 500);
		arm.servoMove(10, 150, 500);
		delay(500);
		arm.servoMove(8, 130, 500);
		arm.servoMove(11, 130, 500);
		delay(500);
		for (int i = 0; i < 6; i++) // 遍历六条腿进行初始化
		{
			arm.servoMove(3 * i + 1, 150, 500); // 初始化第一关节角度为150度
			arm.servoMove(3 * i + 2, 130, 500); // 初始化第二关节角度为130度
			arm.servoMove(3 * i + 3, 140, 500); // 初始化第三关节角度为40度
		}

		delay(500); // 延时等待舵机移动完成
	}
	// 初始化MPU PID控制器
	mpu_pid_x.Init(MPU_X_PID_KP, MPU_X_PID_KI, MPU_X_PID_KD, CIR_OFF);
	mpu_pid_y.Init(MPU_Y_PID_KP, MPU_Y_PID_KI, MPU_Y_PID_KD, CIR_OFF);
	mpu_pid_z.Init(MPU_Y_PID_KP, MPU_Y_PID_KI, MPU_Y_PID_KD, CIR_OFF);
	// 初始化速度和位置的一阶低通滤波器
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
void Hexapod::mode_select(uint8_t Mode)
{

	switch (Mode)
	{
	case 1:
		this->mode = HEXAPOD_MOVE;
		break;
	case 2:
		this->mode = HEXAPOD_BODY_ANGEL_CONTROL;
		break;
	case 3:
		this->mode = HEXAPOD_BODY_POS_CONTROL;
		break;
	default:
		this->mode = HEXAPOD_MOVE;
		break;
	}
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
			Serial.print("超出最大角度");
			theta_temp.angle[0] += 2 * PI; // 调整角度到0-360度范围内
		}
		int servo_base_index = 0;														// 舵机基础索引
		bool reverse_angle_0 = false, reverse_angle_1 = false, reverse_angle_2 = false; // 关节角度反转标志
		// 根据CRXValue和CRYValue的值调整舵机基础索引和关节角度反转标志
		if ((CRXValue < 0 && CRYValue < 0) || CLXValue > 0) // 如果xy都小于零或角速度小于零，绕中点旋转
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
		else if (CLXValue < 0 && (CRXValue == 0 && CRYValue == 0)) // 如果角速度大于零
		{
			servo_base_index = 3 * i + 1;
			reverse_angle_1 = true;
		}
		// 根据计算出的舵机基础索引和关节角度反转标志，控制舵机运动
		if (this->mode == HEXAPOD_MOVE)
		{
			if(devStatus)
				arm.servoMove(servo_base_index, (reverse_angle_0 ? -1 : 1) * theta_temp.angle[0] * (i < 3 ? leftDev : rightDev) / PI * 180 + 150, round_time);
			else
				arm.servoMove(servo_base_index, (reverse_angle_0 ? -1 : 1) * theta_temp.angle[0] * (i < 3 ? mLeftDev : mRightDev) / PI * 180 + 150, round_time);
			arm.servoMove(servo_base_index + 1, (reverse_angle_1 ? -1 : 1) * theta_temp.angle[1] / PI * 180 + 150, round_time);
			arm.servoMove(servo_base_index + 2, (reverse_angle_2 ? -1 : 1) * theta_temp.angle[2] / PI * 180 + 190, round_time);
		}
		else
		{
			arm.servoMove(servo_base_index, (reverse_angle_0 ? -1 : 1) * theta_temp.angle[0]  / PI * 180 + 150, round_time);
			arm.servoMove(servo_base_index + 1, (reverse_angle_1 ? -1 : 1) * theta_temp.angle[1] / PI * 180 + 150, round_time);
			arm.servoMove(servo_base_index + 2, (reverse_angle_2 ? -1 : 1) * theta_temp.angle[2] / PI * 180 + 190, round_time);
		}
	}
}

void Hexapod::Crab_walking(uint16_t timeDelay) 	// 螃蟹走路默认300
{
  // 135腿抬高
  arm.servoMove(2, 100, timeDelay);
  arm.servoMove(8, 100, timeDelay);
  arm.servoMove(14, 100, timeDelay);
  delay(timeDelay);
  // 246腿后蹬
  arm.servoMove(6, 120,  timeDelay);
  arm.servoMove(12, 160,  timeDelay);
  arm.servoMove(18, 160,  timeDelay);
  delay(timeDelay);
  // 135腿前伸
  arm.servoMove(3, 160,  timeDelay);
  arm.servoMove(9, 160,  timeDelay);
  arm.servoMove(15, 120,  timeDelay);
  delay(timeDelay);
  // 135腿放下
  arm.servoMove(2, 130,  timeDelay);
  arm.servoMove(8, 130,  timeDelay);
  arm.servoMove(14, 130,  timeDelay);
  delay(timeDelay);
  // 246腿抬高
  arm.servoMove(5, 100,  timeDelay);
  arm.servoMove(11, 100,  timeDelay);
  arm.servoMove(17, 100,  timeDelay);
  delay(timeDelay);
  // 135腿后蹬
  arm.servoMove(3, 120,  timeDelay);
  arm.servoMove(9, 120,  timeDelay);
  arm.servoMove(15, 160,  timeDelay);
  delay(timeDelay);
  arm.servoMove(6, 160,  timeDelay);
  // 246腿前伸
  arm.servoMove(12, 120,  timeDelay);
  arm.servoMove(18, 120,  timeDelay);
  delay(timeDelay);
  // 246腿放下
  arm.servoMove(5, 130,  timeDelay);
  arm.servoMove(11, 130,  timeDelay);
  arm.servoMove(17, 130,  timeDelay);
  delay(timeDelay);
}

void UserMpu::mpu_Init()
{
	Wire1.begin(32,33);
	Wire1.setClock(400000);
	mpu.initialize();
	Serial.println(F("Initializing I2C devices..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	devStatus = mpu.dmpInitialize();
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);
	if (devStatus == 0)
	{
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		mpu.PrintActiveOffsets();
		mpu.setDMPEnabled(true);

		Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		Serial.println(F(")..."));
		mpuIntStatus = mpu.getIntStatus();
		Serial.println(F("DMP ready! Waiting for first interrupt..."));

		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

void UserMpu::mpu_cab()
{
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
	{
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	}
	if ((ypr[0] * 180 / M_PI) > 0)
	{
		mLeftDev = abs(ypr[0] * 180 / M_PI) / 10 + 1;
		if (mLeftDev > 2)
		mLeftDev = 2;
		mRightDev = 1;
	}

	else if ((ypr[0] * 180 / M_PI) < 0)
	{
		mRightDev = abs(ypr[0] * 180 / M_PI) / 10 + 1;
		if (mRightDev > 2)
		mRightDev = 2;
		mLeftDev = 1;
	}
}
// 参数为x、y、z
float UserMpu::mpu_Get_angle(char x)
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Serial.print(ypr[0] * 180 / M_PI);  // X轴角度
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180 / M_PI);  // Y轴角度
    // Serial.print("\t");
    // Serial.println(ypr[2] * 180 / M_PI);  //Z轴角度
	if(x == 'x')
		return (ypr[0] * 180 / M_PI);
	else if(x == 'y')
		return (ypr[1] * 180 / M_PI);
	else
		return (ypr[2] * 180 / M_PI);
  }
  else
  	return 0;
}

#endif