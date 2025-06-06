#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <limits>
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

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
class PIDController
{
public:
    // 构造函数，初始化PID参数
    PIDController(float Kp, float Ki, float Kd, float max_output = std::numeric_limits<float>::max())
        : Kp(Kp), Ki(Ki), Kd(Kd), max_output(max_output),
          prev_error(0.0f), integral(0.0f) {}

    // 计算PID输出
    float compute(float error, float dt)
    {
        if (dt <= 0.0f)
        {
            return 0.0f; // 避免除以零
        }

        // 比例项
        float proportional = Kp * error;

        // 积分项（带抗饱和）
        integral += error * dt;
        float integral_term = Ki * integral;

        // 微分项
        float derivative = Kd * (error - prev_error) / dt;
        prev_error = error;

        // 计算总输出
        float output = proportional + integral_term + derivative;

        // 限制输出
        if (output > max_output)
        {
            output = max_output;
            integral -= error * dt; // 抗饱和处理
        }
        else if (output < -max_output)
        {
            output = -max_output;
            integral -= error * dt; // 抗饱和处理
        }

        return output;
    }

    // 重置PID控制器
    void reset()
    {
        prev_error = 0.0f;
        integral = 0.0f;
    }

private:
    float Kp, Ki, Kd; // PID增益
    float max_output; // 最大输出限制
    float prev_error; // 上一次的误差
    float integral;   // 积分项累加值
};


static PIDController pid_x(0.5f, 0.01f, 0.1f, 1.0f);   // x方向（前进）
static PIDController pid_y(0.3f, 0.01f, 0.05f, 0.5f);  // y方向（左侧）
static PIDController pid_yaw(0.6f, 0.01f, 0.1f, 1.0f); // 偏航角
                                                       // 初始位姿和目标位姿（ROS坐标系）
// 角度归一化（ROS标准：逆时针为正）
float normalizeAngleROS(float angle)
{
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    while (angle > static_cast<float>(M_PI))
        angle -= two_pi;
    while (angle < -static_cast<float>(M_PI))
        angle += two_pi;
    return angle;
}

// ROS坐标系下的位姿表示
struct ROSPose
{
    float x;   // 前进方向（机器人前方）
    float y;   // 左侧方向
    float yaw; // 偏航角（逆时针为正）
};

// 计算机器人控制输出（ROS坐标系）
void computeROSControl(const ROSPose &current, const ROSPose &target,
                       PIDController &pid_x, PIDController &pid_y, PIDController &pid_yaw,
                       float dt, float &linear_x, float &linear_y, float &angular_z)
{
    // 计算位置误差（ROS坐标系）
    float error_x = target.x - current.x;
    float error_y = target.y - current.y;

    // 计算角度误差（ROS标准：逆时针为正）
    float error_yaw = normalizeAngleROS(target.yaw - current.yaw);

    // 计算控制输出
    linear_x = pid_x.compute(error_x, dt);      // 前进速度
    linear_y = pid_y.compute(error_y, dt);      // 左侧移动速度
    angular_z = pid_yaw.compute(error_yaw, dt); // 角速度（逆时针为正）
}

class speed_couculate_node : public rclcpp::Node
{



public:
    speed_couculate_node() : Node("speed_couculate_node")
    {
        Subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10,
            std::bind(&speed_couculate_node::goal_recieve_callback, this, _1));
        Subscription_2 = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
            std::bind(&speed_couculate_node::speed_couculate_callback, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void goal_recieve_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        goal_x = msg->pose.position.x;
        goal_y = msg->pose.position.y;
        goal_w = getYawFromQuaternion(msg->pose.orientation);
        RCLCPP_INFO(this->get_logger(), "接收到的目标点数据是：(%.2f,%.2f)", goal_x, goal_y);
        RCLCPP_INFO(this->get_logger(), "偏航角：%.2f", goal_w);
        flag_now+=1;
    }
    void speed_couculate_callback(nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        
        if(flag_now!=flag_before)
        {
            now_x = msg->pose.pose.position.x;
            now_y = msg->pose.pose.position.y;
            now_w = getYawFromQuaternion(msg->pose.pose.orientation);
            RCLCPP_INFO(this->get_logger(), "当前的位姿为:%.2f,%.2f,%.2f", now_x, now_y, now_w);
            // 控制循环
            ROSPose current_pose = {now_x, now_y, now_w};
            ROSPose target_pose = {goal_x, goal_y, goal_w};
            float dt = 0.1f;
            if(flag_now!=flag_before)
            {
                float linear_x, linear_y, angular_z;

                computeROSControl(current_pose, target_pose,
                                  pid_x, pid_y, pid_yaw,
                                  dt, linear_x, linear_y, angular_z);

                // 更新位姿（简单积分模型）
                //current_pose.x += linear_x * dt * cosf(current_pose.yaw) - linear_y * dt * sinf(current_pose.yaw);
                //current_pose.y += linear_x * dt * sinf(current_pose.yaw) + linear_y * dt * cosf(current_pose.yaw);
                //current_pose.yaw = normalizeAngleROS(current_pose.yaw + angular_z * dt);
                geometry_msgs::msg::Twist speed;
                speed.linear.x = linear_x*cos(angular_z)+linear_y*sin(angular_z);
                speed.linear.y = linear_y*cos(angular_z)-linear_x*sin(angular_z);
                speed.angular.z = angular_z;
                RCLCPP_INFO(this->get_logger(),"发布速度信息：%.2f,%.2f,%.2f",linear_x,linear_y,angular_z);
                // 发布速度数据
                publisher_->publish(speed);

                // 检查是否到达目标
                float pos_error = hypotf(target_pose.x - current_pose.x,
                                         target_pose.y - current_pose.y);
                float yaw_error = fabs(normalizeAngleROS(target_pose.yaw - current_pose.yaw));

                if (pos_error < 0.05f && yaw_error < 0.02f)
                {
                    std::cout << "Goal reached!" << std::endl;
                    flag_before=flag_now;
                    //break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
            }
            else
            {
                //发布速度为0，维持系统稳定
                geometry_msgs::msg::Twist speed;
                speed.linear.x = 0;
                speed.linear.y = 0;
                speed.angular.z =0;
                RCLCPP_INFO(this->get_logger(),"发布速度信息：%.2f,%.2f,%.2f",0,0,0);
            }
            
        }
        
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
    auto speed_couculate_node1=std::make_shared<speed_couculate_node>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(speed_couculate_node1);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}