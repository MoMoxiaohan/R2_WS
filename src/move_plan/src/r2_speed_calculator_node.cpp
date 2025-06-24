#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <limits>
#include "nav_msgs/msg/odometry.hpp"
#include "pid.h"
#include"std_msgs/msg/float32.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
// PID
PID_Controller pid_controller;
pid_type_def pid_yaw;
pid_type_def pid_pos_x;
pid_type_def pid_pos_y;
fp32 motor_yaw_pid[3] = {1.5, 0.001, 0.5};
fp32 motor_pos_x_pid[3] = {1, 0, 0};
fp32 motor_pos_y_pid[3] = {1, 0, 0};
// 保存点位信息
typedef struct pose
{
    float x, y, w, rpm=0;
} pose;
// 从四元数获取偏航角（yaw）
float getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat)
{
    tf2::Quaternion tf_quat(
        quat.x,
        quat.y,
        quat.z,
        quat.w);
    tf2::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return static_cast<float>(yaw);
}

// 从偏航角创建四元数
geometry_msgs::msg::Quaternion createQuaternionFromYaw(float yaw)
{
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0.0, 0.0, yaw);
    geometry_msgs::msg::Quaternion quat;
    quat.x = tf_quat.x();
    quat.y = tf_quat.y();
    quat.z = tf_quat.z();
    quat.w = tf_quat.w();
    return quat;
}
// PID
void PID_Controller::All_Device_Init(void)
{
    this->core.PID_Init(&pid_yaw, PID_POSITION, motor_yaw_pid, 1.5, 0.1, 0.002);
    this->core.PID_Init(&pid_pos_x, PID_POSITION, motor_pos_x_pid, 1.5, 0.5, 0.01);
    this->core.PID_Init(&pid_pos_y, PID_POSITION, motor_pos_y_pid, 1.5, 0.5, 0.01);
}

fp32 PID_Controller::SENSORS::Yaw_Realize(fp32 set_yaw)
{
    pid_controller.core.PID_Calc(&pid_yaw, this->Now_Pos_Yaw, set_yaw);
    return pid_yaw.out;
}

fp32 PID_Controller::SENSORS::Pos_X_Realize(fp32 set_pos_x)
{
    pid_controller.core.PID_Calc(&pid_pos_x, this->Now_Pos_X, set_pos_x);
    return pid_pos_x.out;
}

fp32 PID_Controller::SENSORS::Pos_Y_Realize(fp32 set_pos_y)
{
    pid_controller.core.PID_Calc(&pid_pos_y, this->Now_Pos_Y, set_pos_y);
    return pid_pos_y.out;
}

class speed_couculate_node : public rclcpp::Node
{

public:
    speed_couculate_node() : Node("r2_speed_calculator_node")
    {

        poses_init();
        Subscription_2 = this->create_subscription<nav_msgs::msg::Odometry>("/r2/aft_mapped_to_init", 10,
                                                                            std::bind(&speed_couculate_node::speed_couculate_callback, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/r2/cmd_vel", 10);
        publisher_2=this->create_publisher<std_msgs::msg::Float32>("/r2/rpm",10);
    }

private:
    void poses_init()
    {
        // 初始化所有已经记录的点
        this->poses[0].x = 13.70;
        this->poses[0].y = -3.19;
        this->poses[0].w = 0, this->poses[0].rpm = 0;

        this->poses[1].x = 12.16;
        this->poses[1].y = -0.20;
        this->poses[1].w = -1.13, this->poses[1].rpm = 0;

        this->poses[2].x = 12.20;
        this->poses[2].y = -0.61;
        this->poses[2].w = -1.09, this->poses[2].rpm = 0;

        this->poses[3].x = 12.22;
        this->poses[3].y = -0.94;
        this->poses[3].w = -1.04, this->poses[3].rpm = 0;

        this->poses[4].x = 12.22;
        this->poses[4].y = -1.33;
        this->poses[4].w = -0.97, this->poses[4].rpm = 0;

        this->poses[5].x = 12.23;
        this->poses[5].y = -1.64;
        this->poses[5].w = -0.90, this->poses[5].rpm = 0;

        this->poses[6].x = 12.21;
        this->poses[6].y = -2.07;
        this->poses[6].w = -0.76, this->poses[6].rpm = 0;

        this->poses[7].x = 12.19;
        this->poses[7].y = -2.40;
        this->poses[7].w = -0.63, this->poses[7].rpm = 0;

        this->poses[8].x = 12.17;
        this->poses[8].y = -2.80;
        this->poses[8].w = -0.43, this->poses[8].rpm = 0;

        this->poses[9].x = 12.14;
        this->poses[9].y = -3.15;
        this->poses[9].w = -0.23, this->poses[9].rpm = 0;

        this->poses[10].x = 12.22;
        this->poses[10].y = -3.58;
        this->poses[10].w = 0.04, this->poses[10].rpm = 0;

        this->poses[11].x = 12.22;
        this->poses[11].y = -3.93;
        this->poses[11].w = 0.26, this->poses[11].rpm = 0;

        this->poses[12].x = 12.28;
        this->poses[12].y = -4.32;
        this->poses[12].w = 0.51, this->poses[12].rpm = 0;

        this->poses[13].x = 12.33;
        this->poses[13].y = -4.68;
        this->poses[13].w = 0.70, this->poses[13].rpm = 0;

        this->poses[14].x = 12.38;
        this->poses[14].y = -5.11;
        this->poses[14].w = 0.87, this->poses[14].rpm = 0;

        this->poses[15].x = 12.27;
        this->poses[15].y = -5.47;
        this->poses[15].w = 0.93, this->poses[15].rpm = 0;

        this->poses[16].x = 12.25;
        this->poses[16].y = -5.88;
        this->poses[16].w = 1.01, this->poses[16].rpm = 0;

        this->poses[17].x = 12.27;
        this->poses[17].y = -6.21;
        this->poses[17].w = 1.07, this->poses[17].rpm = 0;

        this->poses[18].x = 12.25;
        this->poses[18].y = -6.55;
        this->poses[18].w = 1.11, this->poses[18].rpm = 0;

        this->poses[19].x = 11.85;
        this->poses[19].y = -6.55;
        this->poses[19].w = 1.02, this->poses[19].rpm = 0;

        this->poses[20].x = 11.87;
        this->poses[20].y = -6.16;
        this->poses[20].w = 0.96, this->poses[20].rpm = 0;

        this->poses[21].x = 11.83;
        this->poses[21].y = -5.82;
        this->poses[21].w = 0.88, this->poses[21].rpm = 0;

        this->poses[22].x = 11.77;
        this->poses[22].y = -5.40;
        this->poses[22].w = 0.77, this->poses[22].rpm = 0;

        this->poses[23].x = 11.77;
        this->poses[23].y = -5.01;
        this->poses[23].w = 0.65, this->poses[23].rpm = 0;

        this->poses[24].x = 11.72;
        this->poses[24].y = -4.64;
        this->poses[24].w = 0.51, this->poses[24].rpm = 0;

        this->poses[25].x = 11.68;
        this->poses[25].y = -4.28;
        this->poses[25].w = 0.36, this->poses[25].rpm = 0;

        this->poses[26].x = 11.63;
        this->poses[26].y = -3.89;
        this->poses[26].w = 0.18, this->poses[26].rpm = 0;

        this->poses[27].x = 11.60;
        this->poses[27].y = -3.51;
        this->poses[27].w = 0.00, this->poses[27].rpm = 0;

        this->poses[28].x = 11.55;
        this->poses[28].y = -3.13;
        this->poses[28].w = -0.18, this->poses[28].rpm = 0;

        this->poses[29].x = 11.53;
        this->poses[29].y = -2.73;
        this->poses[29].w = -0.35, this->poses[29].rpm = 0;

        this->poses[30].x = 11.50;
        this->poses[30].y = -2.36;
        this->poses[30].w = -0.48, this->poses[30].rpm = 0;

        this->poses[31].x = 11.49;
        this->poses[31].y = -1.96;
        this->poses[31].w = -0.60, this->poses[31].rpm = 0;

        this->poses[32].x = 11.47;
        this->poses[32].y = -1.58;
        this->poses[32].w = -0.71, this->poses[32].rpm = 0;

        this->poses[33].x = 11.44;
        this->poses[33].y = -1.22;
        this->poses[33].w = -0.78, this->poses[33].rpm = 0;

        this->poses[34].x = 11.38;
        this->poses[34].y = -0.82;
        this->poses[34].w = -0.85, this->poses[34].rpm = 0;

        this->poses[35].x = 11.35;
        this->poses[35].y = -0.49;
        this->poses[35].w = -0.90, this->poses[35].rpm = 0;

        this->poses[36].x = 10.35;
        this->poses[36].y = -0.52;
        this->poses[36].w = -0.72, this->poses[36].rpm = 0;

        this->poses[37].x = 10.35;
        this->poses[37].y = -0.91;
        this->poses[37].w = -0.66, this->poses[37].rpm = 0;

        this->poses[38].x = 10.31;
        this->poses[38].y = -1.29;
        this->poses[38].w = -0.58, this->poses[38].rpm = 0;

        this->poses[39].x = 10.23;
        this->poses[39].y = -1.61;
        this->poses[39].w = -0.50, this->poses[39].rpm = 0;

        this->poses[40].x = 10.27;
        this->poses[40].y = -1.99;
        this->poses[40].w = -0.42, this->poses[40].rpm = 0;

        this->poses[41].x = 10.18;
        this->poses[41].y = -2.38;
        this->poses[41].w = -0.31, this->poses[41].rpm = 0;

        this->poses[42].x = 10.23;
        this->poses[42].y = -2.73;
        this->poses[42].w = -0.22, this->poses[42].rpm = 0;

        this->poses[43].x = 10.21;
        this->poses[43].y = -3.11;
        this->poses[43].w = -0.12, this->poses[43].rpm = 0;

        this->poses[44].x = 10.19;
        this->poses[44].y = -3.50;
        this->poses[44].w = -0.01, this->poses[44].rpm = 0;

        this->poses[45].x = 10.17;
        this->poses[45].y = -3.84;
        this->poses[45].w = 0.09, this->poses[45].rpm = 0;

        this->poses[46].x = 10.18;
        this->poses[46].y = -4.21;
        this->poses[46].w = 0.19, this->poses[46].rpm = 0;

        this->poses[47].x = 10.19;
        this->poses[47].y = -4.58;
        this->poses[47].w = 0.29, this->poses[47].rpm = 0;

        this->poses[48].x = 10.22;
        this->poses[48].y = -4.97;
        this->poses[48].w = 0.39, this->poses[48].rpm = 0;


        this->poses[49].x = 10.19;
        this->poses[49].y = -5.34;
        this->poses[49].w = 0.47, this->poses[49].rpm = 0;

        this->poses[50].x = 10.23;
        this->poses[50].y = -5.70;
        this->poses[50].w = 0.56, this->poses[50].rpm = 0;

        this->poses[51].x = 10.23;
        this->poses[51].y = -6.06;
        this->poses[51].w = 0.63, this->poses[51].rpm = 0;

        this->poses[52].x = 10.23;
        this->poses[52].y = -6.42;
        this->poses[52].w = 0.69, this->poses[52].rpm = 0;

        this->poses[53].x = 11.24;
        this->poses[53].y = -5.96;
        this->poses[53].w = 0.78, this->poses[53].rpm = 0;

        this->poses[54].x = 11.06;
        this->poses[54].y = -1.23;
        this->poses[54].w = -0.71, this->poses[54].rpm = 0;


        this->poses[55].x = 9.25;
        this->poses[55].y = -6.61;
        this->poses[55].w = 0.60, this->poses[55].rpm = 0;

        this->poses[56].x = 9.19;
        this->poses[56].y = -5.34;
        this->poses[56].w = 0.38, this->poses[56].rpm = 0;

        this->poses[57].x = 9.06;
        this->poses[57].y = -3.71;
        this->poses[57].w = 0.04, this->poses[57].rpm = 0;

        this->poses[58].x = 8.91;
        this->poses[58].y = -2.28;
        this->poses[58].w = -0.25, this->poses[58].rpm = 0;

        this->poses[59].x = 8.74;
        this->poses[59].y = -1.12;
        this->poses[59].w = -0.45, this->poses[59].rpm = 0;

        this->poses[60].x = 7.46;
        this->poses[60].y = -0.42;
        this->poses[60].w = -0.46, this->poses[60].rpm = 0;

        this->poses[61].x = 7.29;
        this->poses[61].y = -2.15;
        this->poses[61].w = -0.21, this->poses[61].rpm = 0;

        this->poses[62].x = 7.29;
        this->poses[62].y = -4.45;
        this->poses[62].w = 0.14, this->poses[62].rpm = 0;

        this->poses[63].x = 6.90;
        this->poses[63].y = -6.02;
        this->poses[63].w = 0.35, this->poses[63].rpm = 0;

        this->poses[64].x = 5.98;
        this->poses[64].y = -3.51;
        this->poses[64].w = 0.00, this->poses[64].rpm = 0;

        this->poses[65].x = 4.20;
        this->poses[65].y = -6.14;
        this->poses[65].w = 0.27, this->poses[65].rpm = 0;

    }
    int findNearestPose(const pose *poses, int num_poses, float now_x, float now_y)
    {
        if (!poses || num_poses <= 0)
            return -1;

        int nearest_idx = 0;
        float min_dist_sq = std::numeric_limits<float>::max();

        for (int i = 0; i < num_poses; ++i)
        {
            float dx = poses[i].x - now_x;
            float dy = poses[i].y - now_y;
            float dist_sq = dx * dx + dy * dy; // 避免 sqrt 计算

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                nearest_idx = i;
            }
        }
        return nearest_idx;
    }
    void speed_couculate_callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // if(flag_now!=flag_before)
        // {

        now_x = msg->pose.pose.position.x;
        now_y = msg->pose.pose.position.y;
        now_w = getYawFromQuaternion(msg->pose.pose.orientation);
        RCLCPP_INFO(this->get_logger(), "当前的位姿为:%.2f,%.2f,%.2f", now_x, now_y, now_w);
        float dt = 0.1;
        this->index=findNearestPose(poses,66,now_x,now_y);
        float linear_x, linear_y, angular_z;
        pid_controller.sensors.Now_Pos_X = now_x;
        pid_controller.sensors.Now_Pos_Y = now_y;
        pid_controller.sensors.Now_Pos_Yaw = now_w;
        linear_x = pid_controller.sensors.Pos_X_Realize(poses[this->index].x);
        linear_y = pid_controller.sensors.Pos_Y_Realize(poses[this->index].y);
        angular_z = pid_controller.sensors.Yaw_Realize(poses[this->index].w);

        geometry_msgs::msg::Twist speed;
        speed.linear.x = linear_x * cos(now_w) + linear_y * sin(now_w);
        speed.linear.y = -1 * linear_x * sin(now_w) + linear_y * cos(now_w);
        speed.angular.z = angular_z;


        RCLCPP_INFO(this->get_logger(), "发布速度信息：%.2f,%.2f,%.2f", speed.linear.x, speed.linear.y, speed.angular.z);
        RCLCPP_INFO(this->get_logger(),"当前最近的点是%.2f,%.2f rpm为%.2f",poses[index].x,poses[index].y,poses[index].rpm);
        // 发布速度数据
        publisher_->publish(speed);
        std_msgs::msg::Float32 rpm;
        rpm.data=poses[index].rpm;
        publisher_2->publish(rpm);

        // 检查是否到达目标
        // float pos_error = hypotf(goal_x - now_x,
        //                          goal_y - now_y);
        // float yaw_error = fabs(goal_w - now_w);

        // if (pos_error < 0.05f && yaw_error < 0.02f)
        // {
        //     std::cout << "Goal reached!" << std::endl;
        //     flag_before=flag_now;
        //     //break;
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));

    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Subscription_2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_2;
    int flag_now = 0;
    int flag_before = 0;
    float goal_x, goal_y, goal_w;
    float now_x, now_y, now_w;
    int index;
    pose poses[66];
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    pid_controller.All_Device_Init();
    auto speed_couculate_node1 = std::make_shared<speed_couculate_node>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(speed_couculate_node1);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}