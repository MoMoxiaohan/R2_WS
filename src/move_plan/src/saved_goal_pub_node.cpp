#include"rclcpp/rclcpp.hpp"
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

class saved_goal_pub_node:public rclcpp::Node
{
public:
saved_goal_pub_node():Node("saved_goal_pub_node")
{
    //goal_0 original
    poses[0].x=0;poses[0].y=0; poses[0].w=0;
    //goal_1
    poses[1].x=3;poses[1].y=-3;poses[1].w=0;
    //goal_2
    poses[2].x=1;poses[2].y=0;poses[2].w=-3.14;
    //goal_3
    poses[3].x=1;poses[3].y=-1;poses[3].w=0.5;
    publisher_=this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose",10);
}
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
private:
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
pose poses[10];

};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto saved_goal_pub_node1=std::make_shared<saved_goal_pub_node>();
    int num;
    while(1)
    {
        std::cin>>num;
        if(num==-1)
        {
            break;
        }
        else
        {
            saved_goal_pub_node1->pub_func(num);
        }
    }
    flag:

    rclcpp::shutdown();

    return 0;
}