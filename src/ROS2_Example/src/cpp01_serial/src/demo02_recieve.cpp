#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <cstdint>
#include <string>
#include "cpp01_serial/serial_pack.h"
extern std::vector<uint8_t> data_buffer;


class Serial_Node : public rclcpp::Node
{
public:
  Serial_Node() : Node("serial_recieve_node_cpp")
  {
    // 等设备准备好再初始化
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // 串口设备名（根据实际设备调整）
    const std::string device_name = "/dev/pts/6";

    RCLCPP_INFO(this->get_logger(), "Serial port Node Open!");
    // 创建串口配置对象
    // 波特率115200；不开启流控制；无奇偶效验；停止位1。
    drivers::serial_driver::SerialPortConfig config(
        115200,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);
    
    // 初始化串口
    try
    {
      //开1个线程去处理异步
      io_context_ = std::make_shared<drivers::common::IoContext>(1);
      // 初始化 serial_driver_
      serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
      serial_driver_->init_port(device_name, config);
      serial_driver_->port()->open();
      
      RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
      RCLCPP_INFO(this->get_logger(), "Using device: %s", serial_driver_->port().get()->device_name().c_str());
      RCLCPP_INFO(this->get_logger(), "Baud_rate: %d", config.get_baud_rate());
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
      return;
    }

    // // 设置发送定时器
    // transmit_timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(100),
    //     std::bind(&Serial_Node::send_message_timer_callback, this));

    async_receive_message();   //进入异步接收
  }


private:

  void send_message_timer_callback()
  {
    bool bool_buffer[] = {1, 0, 1, 0};
    int8_t int8_buffer[] = {0x11,0x22};
    int16_t int16_buffer[] = {2000,6666};
    // int32_t int32_buffer[] = {305419896};
    fp32 fp32_buffer[] = {3.5};

    //由于ROS2中node为局部变量，所以只能在node中调用send函数，所以Serial_Transmit只负责处理data_buffer。
    serial_pack_.tx.Data_Pack(0x00, 
                                 nullptr, 0,
                                 nullptr, 0,
                                 nullptr, 0,
                                 nullptr, 0,
                                 fp32_buffer, sizeof(fp32_buffer) / sizeof(fp32));

    auto port = serial_driver_->port();

    try
    {
      size_t bytes_size = port->send(data_buffer);
    //   RCLCPP_INFO(this->get_logger(), "Transmitted: ");
    //   for (size_t i = 0; i < data_buffer.size(); ++i) 
    //   {
    //     RCLCPP_INFO(this->get_logger(), "0x%02X ", data_buffer[i]);  //以十六进制格式打印每个字节
    //   }
    //   RCLCPP_INFO(this->get_logger(), "(%ld bytes)", bytes_size);
    }
    catch(const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port:%s",ex.what());
    }
  }

  void async_receive_message()  //创建一个函数更方便重新调用
  {
    auto port = serial_driver_->port();

    // 设置接收回调函数
    port->async_receive([this](const std::vector<uint8_t> &data,const size_t &size) 
    {
        if (size > 0)
        {
          for(int16_t i = 0;i < (int16_t)size;++i)
          {
            uint8_t data_buffer = data[i];
            // 处理接收到的数据
            serial_pack_.rx.Data_Analysis(
              &data_buffer,
              0x01,
            0,
            0,
            0,
            0,
            3);
          }
          //由于在ROS2中，node是局部变量，所以发布方只能在node类里，故Data_Apply不写任何东西，直接在接收下面的回调函数里实现功能。
          if(serial_pack_.rx.data.cmd == 0x01)
          {
            RCLCPP_INFO(this->get_logger(),"接收到的浮点形是:%.2f,%.2f,%.2f",serial_pack_.rx.data.fp32_buffer[0],serial_pack_.rx.data.fp32_buffer[1],serial_pack_.rx.data.fp32_buffer[2]);
            // for(auto i:serial_pack_.rx.data.fp32_buffer)
            // {
            //   RCLCPP_INFO(this->get_logger(),"接收到的浮点形是:%.2f",i);
            // }
          }

        }
        // 继续监听新的数据
        async_receive_message();
    }
    );
  }

  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::shared_ptr<drivers::common::IoContext> io_context_;
  rclcpp::TimerBase::SharedPtr transmit_timer_;
  std::vector<uint8_t> transmit_data_buffer = std::vector<uint8_t>(1024); // 发送缓冲区
  std::vector<uint8_t> receive_data_buffer = std::vector<uint8_t>(1024);  // 接收缓冲区
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Serial_Node>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}