#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
class goal_recieve_node:public rclcpp::Node
{
public:
    goal_recieve_node():Node("goal_recieve_node")
    {
        Subscription_=this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",10,std::bind(&goal_recieve_node::recieve_goal_callback,this,_1));
        RCLCPP_INFO(this->get_logger(),"创建目标接受节点");

    }

private:
    void recieve_goal_callback(const geometry_msgs::msg::PoseStamped &msg)const
    {
        auto x=msg.pose.position.x;
        auto y=msg.pose.position.y;
        RCLCPP_INFO(this->get_logger(),"接收到的目标点数据是：(%.2f,%.2f)",x,y);
        RCLCPP_INFO(this->get_logger(),"偏航角：");
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr Subscription_;
};
int main(int argc,char** argv)
{

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<goal_recieve_node>());
    rclcpp::shutdown();
    return 0;
}