#include "serialport.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cstdint>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_msgs/msg/float32.hpp"
// 枚举协议
typedef enum 
{
    ASK_FIXED_POS_LAUNCH_SPEED = 0x01,//1请求定点投篮
    ALLOW_FIXED_LAUNCH =0x02,                  //2上位机返回定点投篮发射参数（Vx，Vy，Vomega，speed_rpm)
    ASK_CATCH_BALL=0x03,//3
    ALLOW_CATCH_BALL=0x04,//4
    ASK_HANDOFF=0x05,//5
    ALLOW_HANDOFF=0x06,//6                         
    UPPER_OK = 0xff   //7                   //上位机收到请求(可以不用)
}CMD_PACKAGE;
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
void float2array(float num1, float num2, float num3, float num4, uint8_t *array)
{
    memcpy(array, &num1, 4);
    memcpy(array + 4, &num2, 4);
    memcpy(array + 8, &num3, 4);
    memcpy(array + 12, &num4, 4);
}
class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node")
    {
        SerialPort1 = std::make_shared<SerialPort>();
        SerialPort1->init("/dev/Lowerport", 115200);
   
        transmit_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SerialNode::send_message_timer_callback, this));
        Subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/r2/cmd_vel", 10,
                                                                             std::bind(&SerialNode::speed_recieve_callback, this, std::placeholders::_1));
        Subscription_2 = this->create_subscription<std_msgs::msg::Float32>("/r2/omega", 10,
                                                                           std::bind(&SerialNode::pass_omega_callback, this, std::placeholders::_1));
        Subscription_3 = this->create_subscription<std_msgs::msg::Float32>("/r2/rpm", 10,
                                                                           std::bind(&SerialNode::rpm_callback, this, std::placeholders::_1));
        Subscription_4 = this->create_subscription<std_msgs::msg::Float32>("/r2/aim_omega", 10,
                                                                            std::bind(&SerialNode::aim_omega_callback, this, std::placeholders::_1));
        SerialPort1->startAsyncRead();
    }

private:


    void pass_omega_callback(std_msgs::msg::Float32::SharedPtr msg2)
    {
        this->pass_omega = msg2->data;
    }
    void speed_recieve_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        this->liner_x = float(msg->linear.x);
        this->liner_y = float(msg->linear.y);
        this->angular_z = float(msg->angular.z);
    }
    void rpm_callback(std_msgs::msg::Float32::SharedPtr msg3)
    {
        this->rpm=msg3->data;
    }
    void aim_omega_callback(std_msgs::msg::Float32::SharedPtr msg4)
    {
        this->aim_omega=msg4->data;
    }
    void send_message_timer_callback()
    {
        // 导航状态机
        this->flag = SerialPort1->got_cmd();
        switch (flag)
        {
        case 20:
            break;
        case ASK_FIXED_POS_LAUNCH_SPEED:
            float2array(this->liner_x, this->liner_y, this->angular_z, this->rpm, data_auto);
            RCLCPP_INFO(this->get_logger(), "发送的数据是:%.2f %.2f %.2f %.2f", this->liner_x, this->liner_y, this->angular_z,this->rpm);
            SerialPort1->Send_Cmd_Data(ALLOW_FIXED_LAUNCH, data_auto, 16);
            RCLCPP_INFO(this->get_logger(), "现在的指令是 %d", this->flag);
            break;
        case ASK_CATCH_BALL:
            // memcpy(data_pass,&this->pass_omega,4);
            float2array(0,0,this->aim_omega,this->pass_omega,data_auto);
            SerialPort1->Send_Cmd_Data(ALLOW_CATCH_BALL,data_auto,16);
            RCLCPP_INFO(this->get_logger(),"现在的指令是%d",this->flag);
            RCLCPP_INFO(this->get_logger(),"发送的数据是：%.2f,%.2f",this->aim_omega,this->pass_omega);
            break;
        case ASK_HANDOFF:
            //memcpy(data_pass,&this->pass_omega,4);
            float2array(this->pass_omega,0,0,0,data_auto);
            SerialPort1->Send_Cmd_Data(ALLOW_HANDOFF,data_auto,16);
            RCLCPP_INFO(this->get_logger(),"发送的数据是：%.2f",this->pass_omega);
            break;
        case UPPER_OK:
            break;
        default:
            break;
        }
    }

    std::shared_ptr<SerialPort> SerialPort1;
    rclcpp::TimerBase::SharedPtr transmit_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr Subscription_2;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr Subscription_3;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr Subscription_4;
    uint8_t data_auto[16];
    uint8_t data_pass[4];
    float liner_x = 0, liner_y = 0, angular_z = 0;
    float test_num1 = 0, test_num2 = 0, test_num3 = 0;
    float pass_omega = 0;
    float rpm=0;
    int flag = 0;
    float aim_omega=0;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}