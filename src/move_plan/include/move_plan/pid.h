#ifndef __PID_H_
#define __PID_H_

// #include<iostream>
#define LimitMax(input, max)   \
{                          		 \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
}


typedef enum
{
    PID_POSITION = 0,
    PID_DELTA
}PID_MODE;
typedef float fp32;
typedef unsigned char uint8_t;
typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

	fp32 deadband_threshold;

} pid_type_def;

class PID_Controller
{
	public:

		void All_Device_Init(void);
	
		class CORE
		{
			public:
				void PID_Init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout,fp32 deadband_threshold);
				fp32 PID_Calc(pid_type_def *pid, fp32 ref, fp32 set);
				void PID_Clear(pid_type_def *pid);
		}core;
		
		class SENSORS
		{
			public:
				fp32 Yaw_Realize(fp32 set_yaw);
			
				fp32 Pos_X_Realize(fp32 set_pos_x);
				fp32 Pos_Y_Realize(fp32 set_pos_y);
                fp32 Pass_W1(fp32 set_angle,fp32 now_angle);
                fp32 Pass_W2(fp32 set_angle,fp32 now_angle);
                fp32 aim_r2(fp32 set_angle,fp32 now_angle);
                float Now_Pos_X=0,Now_Pos_Y=0,Now_Pos_Yaw=0;
		}sensors;
    

};

#endif
