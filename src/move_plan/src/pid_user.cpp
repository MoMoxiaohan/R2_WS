#include "pid_user.h"
#include "pid.h"

/*PID����������*/
PID_Controller pid_controller;
/***************/

//PID�������(�ṹ��)
pid_type_def pid_yaw;
pid_type_def pid_pos_x;
pid_type_def pid_pos_y;

//��λPID����
fp32 motor_yaw_pid[3] = {1,0,0.1};
fp32 motor_pos_x_pid[3] = {1,0,0};
fp32 motor_pos_y_pid[3] = {1,0,0};

/**
 * @brief       PID�豸��ʼ��
 * @param       void
 * @retval      void
 * @note        ���ｫ���е�PID�豸�Ĳ������г�ʼ��������Kp,Ki,Kd,I_limit(�����޷�),O_limit(���޷�)���������,����ֵ������pid_type_def����С�
 */
void PID_Controller::All_Device_Init(void)
{
  //��λPID
	this->core.PID_Init(&pid_yaw,PID_POSITION,motor_yaw_pid,1,1,0.1);
	this->core.PID_Init(&pid_pos_x,PID_POSITION,motor_pos_x_pid,2,1,0.1);
	this->core.PID_Init(&pid_pos_y,PID_POSITION,motor_pos_y_pid,2,1,0.1);
}

/**
 * @brief       �����PID
 * @param       set_yaw��Ŀ�꺽���
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::SENSORS::Yaw_Realize(fp32 set_yaw)
{
	pid_controller.core.PID_Calc(&pid_yaw,this->Now_Pos_Yaw,set_yaw);
	return pid_yaw.out;
}

/**
 * @brief       X����PID
 * @param       set_pos_x��Ŀ��X����ֵ
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��������λ������ֱ�������������ϵ��X�����ٶȣ�
 */
fp32 PID_Controller::SENSORS::Pos_X_Realize(fp32 set_pos_x)
{
	pid_controller.core.PID_Calc(&pid_pos_x,this->Now_Pos_X,set_pos_x);
	return pid_pos_x.out;
}

/**
 * @brief       Y����PID
 * @param       set_pos_y��Ŀ��Y����ֵ
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��������λ������ֱ�������������ϵ��Y�����ٶȣ�
 */
fp32 PID_Controller::SENSORS::Pos_Y_Realize(fp32 set_pos_y)
{
	pid_controller.core.PID_Calc(&pid_pos_y,this->Now_Pos_Y,set_pos_y);
	return pid_pos_y.out;
}

