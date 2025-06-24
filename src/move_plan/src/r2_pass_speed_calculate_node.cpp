#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/utils.h>
#include <memory>
#include"pid.h"
//PID

PID_Controller pid_controller;
pid_type_def pid_yaw;
pid_type_def pid_aim;
fp32 motor_yaw_pid[3] = {1,0,0.1};
fp32 motor_aim_pid[3]={1.5,0.001,0.5};
//PID
void PID_Controller::All_Device_Init(void)
{
	this->core.PID_Init(&pid_yaw,PID_POSITION,motor_yaw_pid,1.5,0.1,0.01);
    this->core.PID_Init(&pid_aim,PID_POSITION,motor_aim_pid,1.5,0.1,0.002);
}

fp32 PID_Controller::SENSORS::Pass_W2(fp32 set_angle,fp32 now_angle)
{
	pid_controller.core.PID_Calc(&pid_yaw,now_angle,set_angle);
	return pid_yaw.out;
}
fp32 PID_Controller::SENSORS::aim_r2(fp32 set_angle,fp32 now_angle)
{
    pid_controller.core.PID_Calc(&pid_aim,now_angle,set_angle);
    return pid_aim.out;
}

class Robot2Controller : public rclcpp::Node {
public:
    Robot2Controller() : Node("r2_pass_speed_calculate_node") {
        // 订阅机器人1的odom
        sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/r1/aft_mapped_to_init", 10,
            std::bind(&Robot2Controller::robot1_odom_callback, this, std::placeholders::_1));

        // 订阅机器人2的odom
        sub2_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/r2/aft_mapped_to_init", 10,
            std::bind(&Robot2Controller::robot2_odom_callback, this, std::placeholders::_1));

        // 发布角速度控制命令
        omega_pub_ = this->create_publisher<std_msgs::msg::Float32>("/r2/omega", 10);
        //自瞄角速度发布
        aim_pub_=this->create_publisher<std_msgs::msg::Float32>("/r2/aim_omega",10);
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

        //float now_angle=robot2_yaw_;
        float distance=sqrt((robot1_x_-robot2_x_)*(robot1_x_-robot2_x_)+(robot1_y_-robot2_y_)*(robot1_y_-robot2_y_));
        float set_angle=atan((robot2_y_-robot1_y_)/(robot2_x_-robot1_x_));


        //远框
        //float distance_aim=sqrt((13.7-robot2_x_)*(13.7-robot2_x_)+(-3.19-robot2_y_)*(-3.19-robot2_y_));
        //float angle_aim=atan((robot2_y_+3.52)/(robot2_x_-13.73));
        //近框
        std_msgs::msg::Float32 aim_omega;
        float angle_aim=atan((robot2_y_+3.65)/(robot2_x_-0.71));
        if(robot2_y_==-3.54)
        {
            angle_aim=0.0f;
        }
        if(robot2_yaw_<=0)
        {
            aim_omega.data=pid_controller.sensors.aim_r2(angle_aim,robot2_yaw_+M_PI);
        }
        else if(robot2_yaw_>0)
        {
            aim_omega.data=pid_controller.sensors.aim_r2(angle_aim,robot2_yaw_-M_PI);
        }

        RCLCPP_INFO(this->get_logger(),"当前位姿为%.2f %.2f %.2f,目标角度%.2f",this->robot2_x_,robot2_y_,robot2_yaw_,angle_aim);
        
        std_msgs::msg::Float32 omega;
        //std_msgs::msg::Float32 aim_omega;

        
        //omega.data=pid_controller.sensors.Pass_W2(set_angle,now_angle);
        omega.data=sqrt((0.62-robot2_x_)*(0.62-robot2_x_)+(-3.54-robot2_y_)*(-3.54-robot2_y_));
        //aim_omega.data=pid_controller.sensors.aim_r2(angle_aim,robot2_yaw_);
        omega_pub_->publish(omega);
        aim_pub_->publish(aim_omega);
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


    
    // 机器人状态
    double robot1_yaw_=0, robot2_yaw_=0;
    double robot1_x_=0, robot1_y_=0;
    double robot2_x_=0, robot2_y_=0;
    
    // ROS订阅和发布
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr omega_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr aim_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    pid_controller.All_Device_Init();
    auto node = std::make_shared<Robot2Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}