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
//#include "pid_user.h"
#include "pid.h"
using std::placeholders::_1;
using std::placeholders::_2;
//PID
PID_Controller pid_controller;
pid_type_def pid_yaw;
pid_type_def pid_pos_x;
pid_type_def pid_pos_y;
fp32 motor_yaw_pid[3] = {1,0,0.1};
fp32 motor_pos_x_pid[3] = {1,0,0};
fp32 motor_pos_y_pid[3] = {1,0,0};


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
//PID
void PID_Controller::All_Device_Init(void)
{
	this->core.PID_Init(&pid_yaw,PID_POSITION,motor_yaw_pid,1.5,0.1,0.01);
	this->core.PID_Init(&pid_pos_x,PID_POSITION,motor_pos_x_pid,1.5,0.5,0.01);
	this->core.PID_Init(&pid_pos_y,PID_POSITION,motor_pos_y_pid,1.5,0.5,0.01);
}

fp32 PID_Controller::SENSORS::Yaw_Realize(fp32 set_yaw)
{
	pid_controller.core.PID_Calc(&pid_yaw,this->Now_Pos_Yaw,set_yaw);
	return pid_yaw.out;
}

fp32 PID_Controller::SENSORS::Pos_X_Realize(fp32 set_pos_x)
{
	pid_controller.core.PID_Calc(&pid_pos_x,this->Now_Pos_X,set_pos_x);
	return pid_pos_x.out;
}

fp32 PID_Controller::SENSORS::Pos_Y_Realize(fp32 set_pos_y)
{
	pid_controller.core.PID_Calc(&pid_pos_y,this->Now_Pos_Y,set_pos_y);
	return pid_pos_y.out;
}

class speed_couculate_node : public rclcpp::Node
{



public:
    speed_couculate_node() : Node("r2_speed_calculator_node")
    {
        Subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/r2/goal_pose", 10,
            std::bind(&speed_couculate_node::goal_recieve_callback, this, _1));
        Subscription_2 = this->create_subscription<nav_msgs::msg::Odometry>("/r2/aft_mapped_to_init", 10,
            std::bind(&speed_couculate_node::speed_couculate_callback, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/r2/cmd_vel", 10);
    }

private:
    void goal_recieve_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        goal_x = msg->pose.position.x;
        goal_y = msg->pose.position.y;
        goal_w = getYawFromQuaternion(msg->pose.orientation);
        flag_now+=1;
    }
    void speed_couculate_callback(nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        // if(flag_now!=flag_before)
        // {
        
            now_x = msg->pose.pose.position.x;
            now_y = msg->pose.pose.position.y;
            now_w = getYawFromQuaternion(msg->pose.pose.orientation);
            RCLCPP_INFO(this->get_logger(), "接收到的目标点数据是：(%.2f,%.2f)", goal_x, goal_y);
            RCLCPP_INFO(this->get_logger(), "偏航角：%.2f", goal_w);
            RCLCPP_INFO(this->get_logger(), "当前的位姿为:%.2f,%.2f,%.2f", now_x, now_y, now_w);
            float dt=0.1;
            // 控制循环
            // if(flag_now!=flag_before)
            // {
                float linear_x, linear_y, angular_z;
                pid_controller.sensors.Now_Pos_X=now_x;
                pid_controller.sensors.Now_Pos_Y=now_y;
                pid_controller.sensors.Now_Pos_Yaw=now_w;
                linear_x=pid_controller.sensors.Pos_X_Realize(goal_x);
                linear_y=pid_controller.sensors.Pos_Y_Realize(goal_y);
                angular_z=pid_controller.sensors.Yaw_Realize(goal_w);
                // 更新位姿（简单积分模型）
                //current_pose.x += linear_x * dt * cosf(current_pose.yaw) - linear_y * dt * sinf(current_pose.yaw);
                //current_pose.y += linear_x * dt * sinf(current_pose.yaw) + linear_y * dt * cosf(current_pose.yaw);
                //current_pose.yaw = normalizeAngleROS(current_pose.yaw + angular_z * dt);

                //对于全向轮
                geometry_msgs::msg::Twist speed;
                speed.linear.x = linear_x*cos(now_w)+linear_y*sin(now_w);
                speed.linear.y = -1*linear_x*sin(now_w)+linear_y*cos(now_w);
                speed.angular.z = angular_z;

                //对于差速模型
                // geometry_msgs::msg::Twist speed;
                // speed.linear.x = linear_x*cos(now_w)+linear_y*sin(now_w);
                // speed.linear.y = linear_y*cos(now_w)-linear_x*sin(now_w);
                // speed.angular.z = angular_z;

                RCLCPP_INFO(this->get_logger(),"发布速度信息：%.2f,%.2f,%.2f",speed.linear.x,speed.linear.y,speed.angular.z);
                // 发布速度数据
                publisher_->publish(speed);

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
            //}
            
            
        //}
        // else
        // {
        //     //发布速度为0，维持系统稳定
        //     geometry_msgs::msg::Twist speed;
        //     speed.linear.x = 0;
        //     speed.linear.y = 0;
        //     speed.angular.z =0;
        //     RCLCPP_INFO(this->get_logger(),"发布速度信息：%.2f,%.2f,%.2f",0,0,0);
        // }
        
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr Subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Subscription_2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    int flag_now=0;
    int flag_before=0;
    float goal_x, goal_y, goal_w;
    float now_x,now_y,now_w;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    pid_controller.All_Device_Init();
    auto speed_couculate_node1=std::make_shared<speed_couculate_node>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(speed_couculate_node1);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}