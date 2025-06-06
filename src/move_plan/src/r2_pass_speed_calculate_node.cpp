#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/utils.h>
#include <memory>

class Robot1Controller : public rclcpp::Node {
public:
    Robot1Controller() : Node("r2_pass_speed_calculate_node") {
        // PID参数
        kp_ = 0.5;
        ki_ = 0.01;
        kd_ = 0.1;
        integral_ = 0.0;
        prev_error_ = 0.0;

        // 订阅机器人1的odom
        sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot1/odom", 10,
            std::bind(&Robot1Controller::robot1_odom_callback, this, std::placeholders::_1));

        // 订阅机器人2的odom
        sub2_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot2/odom", 10,
            std::bind(&Robot1Controller::robot2_odom_callback, this, std::placeholders::_1));

        // 发布角速度控制命令
        omega_pub_ = this->create_publisher<std_msgs::msg::Float32>("/omega1", 10);

        // 初始化变量
        robot1_yaw_ = 0.0;
        robot2_yaw_ = 0.0;
        robot1_x_ = 0.0;
        robot1_y_ = 0.0;
        robot2_x_ = 0.0;
        robot2_y_ = 0.0;
    }

private:
    void robot1_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot1_x_ = msg->pose.pose.position.x;
        robot1_y_ = msg->pose.pose.position.y;
        robot1_yaw_ = get_yaw_from_quaternion(msg->pose.pose.orientation);
    }

    void robot2_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot2_x_ = msg->pose.pose.position.x;
        robot2_y_ = msg->pose.pose.position.y;
        robot2_yaw_ = get_yaw_from_quaternion(msg->pose.pose.orientation);
        
        // 计算控制命令
        calculate_control();
    }

    void calculate_control() {
        // 计算期望角度（指向机器人2）
        double desired_yaw = atan2(robot2_y_ - robot1_y_, robot2_x_ - robot1_x_);
        
        // 计算角度误差（归一化到[-π, π]）
        double error = desired_yaw - robot1_yaw_;
        error = atan2(sin(error), cos(error));
        
        // PID控制
        integral_ += error;
        double derivative = error - prev_error_;
        double omega = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;
        
        // 发布角速度命令
        std_msgs::msg::Float32 omega_msg;
        omega_msg.data = omega;
        omega_pub_->publish(omega_msg);
    }

    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat) {
        tf2::Quaternion tf_quat(
            quat.x,
            quat.y,
            quat.z,
            quat.w);
        tf2::Matrix3x3 mat(tf_quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        return yaw;
    }

    // PID参数
    double kp_, ki_, kd_;
    double integral_, prev_error_;
    
    // 机器人状态
    double robot1_yaw_, robot2_yaw_;
    double robot1_x_, robot1_y_;
    double robot2_x_, robot2_y_;
    
    // ROS订阅和发布
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr omega_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Robot1Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}