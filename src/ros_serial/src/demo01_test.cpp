#include "serialport.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cstdint>
#include <string>
#include"geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
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
typedef struct pose
{
    float x,y,w;
}pose;
void float2array(float num1,float num2,float num3,uint8_t*array)
{
    memcpy(array,&num1,4);
    memcpy(array+4,&num2,4);
    memcpy(array+8,&num3,4);
}
class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node")
    {
        //初始化所有已经记录的点
        this->poses[0].x=0;this->poses[0].y=0;this->poses[0].w=0;
        this->poses[1].x=1;this->poses[1].y=1;this->poses[1].w=0;
        this->poses[2].x=2;this->poses[2].y=2;this->poses[2].w=0;
        this->poses[3].x=2;this->poses[3].y=4;this->poses[3].w=0;
        this->poses[4].x=2;this->poses[4].y=6;this->poses[4].w=0;
        this->poses[5].x=3;this->poses[5].y=4;this->poses[5].w=0;
        this->poses[6].x=3;this->poses[6].y=6;this->poses[6].w=0;
       
        SerialPort1=std::make_shared<SerialPort>();
        SerialPort1->init("/dev/pts/5", 115200);
        publisher_=this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose",10);
        transmit_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SerialNode::send_message_timer_callback, this));
        Subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
            std::bind(&SerialNode::speed_recieve_callback, this, std::placeholders::_1));
        SerialPort1->startAsyncRead();
    }

private:
    void pub_func(int num)
    {
        geometry_msgs::msg::PoseStamped real_pose;
        real_pose.pose.position.x=poses[num].x;
        real_pose.pose.position.y=poses[num].y;
        real_pose.pose.orientation=createQuaternionFromYaw(poses[num].w);
        publisher_->publish(real_pose);
        RCLCPP_INFO(this->get_logger(),"成功发布目标点%.2f,%.2f,%.2f",real_pose.pose.position.x,real_pose.pose.position.y
        ,getYawFromQuaternion(real_pose.pose.orientation));

    }
    void speed_recieve_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        this->liner_x = float(msg->linear.x);
        this->liner_y = float(msg->linear.y);
        this->angular_z = float(msg->angular.z);
    }
    void send_message_timer_callback()
    {
        float2array(this->test_num1,this->test_num2,this->test_num3,data);
        test_num1+=0.3;
        test_num2+=0.3;
        test_num3+=0.3;
        RCLCPP_INFO(this->get_logger(),"发送的数据是:%.2f %.2f %.2f",this->test_num1,this->test_num2,this->test_num3);
        SerialPort1->Send_Cmd_Data(0,data,12);
        //导航状态机
        if(SerialPort1->got_cmd()!=this->flag)
        {
            this->flag=SerialPort1->got_cmd();
        }
        switch(flag)
        {
            case 20:
                break;
            case 0:
                pub_func(0);
                break;
            case 1:
                pub_func(1);
                break;
            case 2:
                pub_func(2);
                break;
            case 3:
                pub_func(3);
                break;
            case 4:
                pub_func(4);
                break;
            case 5:
                pub_func(5);
                break;
            case 6:
                pub_func(6);
                break;
            default:
                break;
        }
    }
    std::shared_ptr<SerialPort> SerialPort1;
    rclcpp::TimerBase::SharedPtr transmit_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    uint8_t data[12];
    float liner_x = 0, liner_y = 0, angular_z = 0;
    float test_num1=0,test_num2=0,test_num3=0;
    int flag = 0;
    pose poses[10];
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}