#include "serialport.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cstdint>
#include <string>
void float2array(float num1,float num2,float num3,uint8_t*array)
{
    memcpy(array,&num1,4);
    memcpy(array+4,&num2,4);
    memcpy(array+8,&num3,4);
}
class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_recieve_node")
    {
        SerialPort1=std::make_shared<SerialPort>();
        SerialPort1->init("/dev/pts/6", 115200);

        // transmit_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(1000),
        //     std::bind(&SerialNode::send_message_timer_callback, this));
        // Subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
        //     std::bind(&SerialNode::speed_recieve_callback, this, std::placeholders::_1));
        SerialPort1->startAsyncRead();
    }

private:
    void speed_recieve_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        this->liner_x = float(msg->linear.x);
        this->liner_y = float(msg->linear.y);
        this->angular_z = float(msg->angular.z);
    }
    void send_message_timer_callback()
    {
        float2array(this->liner_x,this->liner_y,this->angular_z,data);
        RCLCPP_INFO(this->get_logger(),"发送的数据是:%.2f %.2f %.2f",this->liner_x,this->liner_y,this->angular_z);
        
        SerialPort1->Send_Cmd_Data(0,data,12);
    }
    std::shared_ptr<SerialPort> SerialPort1;
    rclcpp::TimerBase::SharedPtr transmit_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Subscription_;
    uint8_t data[12];
    float liner_x = 0, liner_y = 0, angular_z = 0;
    int flag = 1;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}