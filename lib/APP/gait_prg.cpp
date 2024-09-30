#include "Arduino.h"
#include "gait_prg.h"
#include <cmath>
#include "my_math.h"
using namespace std;

// ȫ�ֱ���
extern uint32_t LegControl_round; // ���ƻغ�

// ����
static Position3 fkine(Thetas thetas);
static Thetas ikine(Position3 &pos);

void Gait_prg::Init()
{
    // �����е���������ʼ�˵�ĩ������
    Pws[0] = fkine(Thetas(PI / 4, THETA_STAND_2, THETA_STAND_3));
    Pws[1] = fkine(Thetas(0, THETA_STAND_2, THETA_STAND_3));
    Pws[2] = fkine(Thetas(-PI / 4, THETA_STAND_2, THETA_STAND_3));
    Pws[3] = fkine(Thetas(3 * PI / 4, THETA_STAND_2, THETA_STAND_3));
    Pws[4] = fkine(Thetas(PI, THETA_STAND_2, THETA_STAND_3));
    Pws[5] = fkine(Thetas(5 * PI / 4, THETA_STAND_2, THETA_STAND_3));
    // Ĭ��վ�����꣬����copyһ��
    memcpy(Pws_default, Pws, sizeof(Position3) * 6);
    // ���������е����ʼ������ڻ��������ĵ�����
    P_legs[0] = Position3(CHASSIS_FRONT_WIDTH / 2, CHASSIS_LEN / 2, 0);
    P_legs[1] = Position3(CHASSIS_WIDTH / 2, 0, 0);
    P_legs[2] = Position3(CHASSIS_FRONT_WIDTH / 2, -CHASSIS_LEN / 2, 0);
    P_legs[3] = Position3(-CHASSIS_FRONT_WIDTH / 2, CHASSIS_LEN / 2, 0);
    P_legs[4] = Position3(-CHASSIS_WIDTH / 2, 0, 0);
    P_legs[5] = Position3(-CHASSIS_FRONT_WIDTH / 2, -CHASSIS_LEN / 2, 0);
}

/*
 * ���˶�����
 */
static Position3 fkine(Thetas thetas)
{
    Position3 position3(cos(thetas.angle[0]) * (LEG_LEN1 + LEG_LEN3 * cos(thetas.angle[1] + thetas.angle[2]) + LEG_LEN2 * cos(thetas.angle[1])),
                        sin(thetas.angle[0]) * (LEG_LEN1 + LEG_LEN3 * cos(thetas.angle[1] + thetas.angle[2]) + LEG_LEN2 * cos(thetas.angle[1])),
                        LEG_LEN3 * sin(thetas.angle[1] + thetas.angle[2]) + LEG_LEN2 * sin(thetas.angle[1]));

    return position3;
}

/*
 * ���˶�����
 */
static Thetas ikine(Position3 &pos)
{
    static Position3 pos1;
    static float R, Lr, alpha_r, alpha1, alpha2;
    pos1 = pos;
    R = sqrt(pow(pos1.x, 2) + pow(pos1.y, 2));
    Lr = sqrt(pow(pos1.z, 2) + pow((R - LEG_LEN1), 2));
    alpha_r = atan(-pos1.z / (R - LEG_LEN1));
    alpha1 = acos((pow(LEG_LEN2, 2) + pow(Lr, 2) - pow(LEG_LEN3, 2)) / (2 * Lr * LEG_LEN2));
    alpha2 = acos((pow(Lr, 2) + pow(LEG_LEN3, 2) - pow(LEG_LEN2, 2)) / (2 * Lr * LEG_LEN3));
    Thetas thetas(atan2(pos1.y, pos1.x), alpha1 - alpha_r, -(alpha1 + alpha2));
    value_limit(thetas.angle[1], MIN_JOINT2_RAD, MAX_JOINT2_RAD);
    value_limit(thetas.angle[2], MIN_JOINT3_RAD, MAX_JOINT3_RAD);
    return thetas;
}

float Gait_prg::move_point()
{
    float fun,m_velocity;
    m_velocity = sqrt(pow(velocity.Vx,2)+pow(velocity.Vy,2));
    fun = (body_pos.x * velocity.Vx + body_pos.y * velocity.Vy)/(m_velocity)*K_W;
    return fun;
}

void Gait_prg::set_body_rotate_angle(Position3 &rotate_angle)
{
    this->rotate_angle = rotate_angle;
}

/*
 *@brief ͨ��������ת�Ƕȣ������е��ĩ��λ��
 *@param point ��ĩ���������ʼ�˵����꣬
 *@param index �ȵı��
 */
Position3 Gait_prg::hexapod_rotate(Position3 &point, uint32_t index)
{
    Position3 retvel;
    retvel.x = cos(rotate_angle.y) * cos(rotate_angle.z) * (P_legs[index].x + point.x) - cos(rotate_angle.y) * sin(rotate_angle.z) * (P_legs[index].y + point.y);
    retvel.y = (cos(rotate_angle.x) * sin(rotate_angle.z) + cos(rotate_angle.z) * sin(rotate_angle.x) * sin(rotate_angle.y)) * (P_legs[index].x + point.x) + (cos(rotate_angle.x) * cos(rotate_angle.z) - sin(rotate_angle.x) * sin(rotate_angle.y) * sin(rotate_angle.z)) * (P_legs[index].y + point.y);
    retvel.z = point.z + (sin(rotate_angle.x) * sin(rotate_angle.z) - cos(rotate_angle.x) * cos(rotate_angle.z) * sin(rotate_angle.y)) * (P_legs[index].x + point.x) + (cos(rotate_angle.z) * sin(rotate_angle.x) + cos(rotate_angle.x) * sin(rotate_angle.y) * sin(rotate_angle.z)) * (P_legs[index].y + point.y);
    retvel = retvel - P_legs[index];
    return retvel;
}

/*
 *@brief ���û����˸߶�
 *@param height �����˵ĸ߶�
 */
void Gait_prg::set_height(float height)
{
    for (int i = 0; i < 6; i++)
    {
        Pws[i].z = Pws_default[i].z + height;
    }
}

/*
 *@brief ���û���������λ��
 *@param body_pos �����˵�����λ��
 */
void Gait_prg::set_body_position(Position3 &body_pos)
{
    this->body_pos = body_pos;
    for (int i = 0; i < 6; i++)
    {
        Pws[i] = Pws_default[i] - body_pos;
    }
}

/*
 *@brief ���û������ٶ�
 *@param velocity �������ٶ�
 */
void Gait_prg::set_velocity(Velocity &velocity)
{
    this->velocity = velocity;
}

/*
 * ����Բ��λ�úͲ�����С�Ѽ�����ִ��ʱ��
 */
void Gait_prg::CEN_and_pace_cal()
{
    // ����Ԥ�������������0
    if (velocity.Vx == 0)
        velocity.Vx += 0.001f;
    if (velocity.Vy == 0)
        velocity.Vy += 0.001f;
    if (velocity.omega == 0)
        velocity.omega += 0.001f;

    if (velocity.omega < 0)
    {
        velocity.Vx = -velocity.Vx;
        velocity.Vy = -velocity.Vy;
    }

    // ����Բ��ģ��
    float module_CEN = K_CEN / velocity.omega * sqrt(pow(velocity.Vx, 2) + pow(velocity.Vy, 2));
    Velocity velocity_s; // ��ת90��
    velocity_s.Vx = -velocity.Vy;
    velocity_s.Vy = velocity.Vx;
    if (velocity_s.Vx >= 0)
        CEN.x = sqrt(pow(module_CEN, 2) / (1 + pow(velocity.Vx, 2) / pow(velocity.Vy, 2)));
    else
        CEN.x = -sqrt(pow(module_CEN, 2) / (1 + pow(velocity.Vx, 2) / pow(velocity.Vy, 2)));
    // ���㲽����С
    float module_speed = pow(pow(velocity.Vx, 3) + pow(velocity.Vy, 3) + pow(velocity.omega, 3), 1.0f / 3);
    if (module_speed > MAX_SPEED)
        module_speed = MAX_SPEED; // �����ٶ�
    R_pace = KR_2 * module_speed;
    // ���㲽��ʱ��
    if (R_pace > MAX_R_PACE)
        this->pace_time = 1000 / (R_pace / MAX_R_PACE); // ��������󲽷���С����С����ʱ��
    else
        this->pace_time = 1000; // ��С����󲽷���С��̶�����ʱ��
    if (R_pace > MAX_R_PACE)
        R_pace = MAX_R_PACE; // ���Ʋ�����С
    CEN.y = -CEN.x * velocity.Vx / velocity.Vy;
}

/*
 * ��̬�滮
 */
void Gait_prg::gait_proggraming()
{
    Position3 Vec_CEN2leg_ends[6];    // Բ�ĵ��Ȳ�ĩ�˵�����
    static float angle_off[6];        // Բ�����е��ĩ�˵ļн�
    static float norm_CEN2legs[6];    // Բ�ĵ���е��ĩ�˵�ģ��
    static float Rp_ratios[6];        // ������е�Ȳ�̬�滮�Ĵ�С����
    Position3 Vec_Leg_Start2CEN_s[6]; // �Ȳ���ʼ�˵�Բ����ʼ�˵�����
    for (int i = 0; i < 6; i++)
    {
        Vec_CEN2leg_ends[i] = Pws[i] + P_legs[i] - CEN;                                         // ����Բ�ĵ�ÿ���Ȳ�ĩ�˵�����
        angle_off[i] = atan2(Vec_CEN2leg_ends[i].y, Vec_CEN2leg_ends[i].x);                     // ����Բ�����е��ĩ�˵ļн�
        norm_CEN2legs[i] = sqrt(pow(Vec_CEN2leg_ends[i].x, 2) + pow(Vec_CEN2leg_ends[i].y, 2)); // ����Բ�����е��ĩ�˵�ģ��
        Vec_Leg_Start2CEN_s[i] = CEN - P_legs[i];                                               // �����Ȳ���ʼ�˵�Բ����ʼ�˵�����
    }
    float max_norm_CEN2legs = 0;
    for (int i = 0; i < 6; i++)
        if (norm_CEN2legs[i] > max_norm_CEN2legs)
            max_norm_CEN2legs = norm_CEN2legs[i]; // ѡ�����ģ��

    static float R_paces[6]; // ������е�ȵĲ���
    for (int i = 0; i < 6; i++)
    {
        Rp_ratios[i] = norm_CEN2legs[i] / max_norm_CEN2legs; // ���������е�Ȳ�̬�滮�Ĵ�С����
        R_paces[i] = Rp_ratios[i] * R_pace;                  // ���������е�Ȳ�̬�Ĵ�С
    }
    float d_theta = 2 * R_paces[0] / norm_CEN2legs[0]; // �����е����һ����Բ�ĹյĽǶȣ������һ�������������
    float step_size = d_theta / (N_POINTS / 2);

    /*********�ȶ���1��3��5����̬�滮***********/
    static float angle_t;   // ���ڼ���õ�ĽǶ�
    static float y_temp;    // ���ڼ���z��߶ȵ���ʱ����
    static Position3 point; // ���ڴ洢ĩ�������
    for (int i = 0; i < 5; i += 2)
    {
        if (LegControl_round < N_POINTS / 2) // 0-9, ���°�Բ
        {
            angle_t = angle_off[i] + d_theta / 2 - step_size * LegControl_round;  // ���������ĽǶ�
            point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t); // ����������x������(����ڻ�е����ʼ��)
            point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t); // ����������y������(����ڻ�е����ʼ��)
            point.z = Pws[i].z;                                                   // ����ǰ�벿�����ŵ��棬��ȡվ��ʱ��z������
        }
        else // 10-19�����ϰ�Բ
        {
            angle_t = angle_off[i] - d_theta / 2 + step_size * (LegControl_round - N_POINTS / 2); // ���������ĽǶ�
            point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t);                 // ����������x������(����ڻ�е����ʼ��)
            point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t);                 // ����������y������(����ڻ�е����ʼ��)
            y_temp = -R_pace + (LegControl_round - N_POINTS / 2) * (R_pace * 4 / N_POINTS);
            // ����Բ�Ĵ�С��Сz��߶�,��Ǩ������ϵ����е��ĩ��,��Ϊվ��ʱz�ᶼ��һ���ģ����������һ��Pw����
            if (R_pace > 0.5f && R_pace < MIN_Z_PACE)
                point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] * 3 + Pws[i].z;
            else
                point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] + Pws[i].z;
        }
        point = hexapod_rotate(point, i);
        actions[i].thetas[LegControl_round] = ikine(point);
    }

    /*********����2��4��6����̬�滮***********/
    for (int i = 1; i <= 5; i += 2)
    {
        if (LegControl_round < N_POINTS / 2) // 0-9, ���ϰ�Բ
        {
            angle_t = angle_off[i] - d_theta / 2 + step_size * LegControl_round;  // ���������ĽǶ�
            point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t); // ����������x������(����ڻ�е����ʼ��)
            point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t); // ����������y������(����ڻ�е����ʼ��)
            y_temp = -R_pace + LegControl_round * (R_pace * 4 / N_POINTS);
            // ����Բ�Ĵ�С��Сz��߶�,��Ǩ������ϵ����е��ĩ��,��Ϊվ��ʱz�ᶼ��һ���ģ����������һ��Pw����
            if (R_pace > 0.5f && R_pace < MIN_Z_PACE)
                point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] * 3 + Pws[i].z;
            else
                point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] + Pws[i].z;
        }
        else // 10-19, ���°�Բ
        {
            angle_t = angle_off[i] + d_theta / 2 - step_size * (LegControl_round - N_POINTS / 2); // ���������ĽǶ�
            point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t);                 // ����������x������(����ڻ�е����ʼ��)
            point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t);                 // ����������y������(����ڻ�е����ʼ��)
            point.z = Pws[i].z;
        }
        point = hexapod_rotate(point, i);
        actions[i].thetas[LegControl_round] = ikine(point);
    }  
}

uint32_t Gait_prg::get_pace_time()
{
    return this->pace_time;
}