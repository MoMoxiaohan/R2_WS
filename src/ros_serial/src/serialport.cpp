#include "../include/ros_serial/serialport.h"

SerialPort::SerialPort() :
    portName("/dev/pts/6"),
    baudRate(115200)
    {
		
    }
bool SerialPort::init(std::string port_name, int baud_rate)
{
    this->portName = port_name;
    this->baudRate = baud_rate;

    return open_port();
}

void SerialPort::runService()
{
    startAsyncRead();
   
}

bool SerialPort::open_port()
{
	drivers::serial_driver::SerialPortConfig config(
        this->baudRate,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);    
	try
    {
      // 开1个线程去处理异步
      io_context_ = std::make_shared<drivers::common::IoContext>(1);
      // 初始化 serial_driver_
      sp = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
      sp->init_port(this->portName, config);
      sp->port()->open();
      RCLCPP_INFO(this->logger_in, "Serial port initialized successfully");
      RCLCPP_INFO(this->logger_in, "Using device: %s", sp->port().get()->device_name().c_str());
      RCLCPP_INFO(this->logger_in, "Baud_rate: %d", config.get_baud_rate());
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->logger_in, "Failed to initialize serial port: %s", ex.what());
      return -1;
    }
	return true;

    
}

void SerialPort::startAsyncRead()
{
    // 设置接收回调函数
    sp->port()->async_receive([this](const std::vector<uint8_t> &data, const size_t &size)
                        {
        if (size > 0)
        {
          for(int16_t i = 0;i < (int16_t)size;++i)
          {
            //uint8_t data_buffer = data[i];

			Receive(data[i]);
            // 处理接收到的数据
          }
        }
        // 继续监听新的数据
       	startAsyncRead(); });
    
}

void SerialPort::close()
{
    return;
}

void SerialPort::SendByte(uint8_t date[])
{
	return;
}
void SerialPort::Send(std::vector<uint8_t>buffer)
{
    sp->port()->send(buffer);
}
void SerialPort::Send_Cmd_Data(uint8_t cmd, const uint8_t* datas, uint8_t len)
{

	uint8_t buf[300], i, cnt = 0;
	uint16_t crc16;	
	buf[cnt++] = 0xA5;
	buf[cnt++] = 0x5A;
	buf[cnt++] = len;
	buf[cnt++] = cmd;
	for (i = 0; i < len; i++)
	{
		buf[cnt++] = datas[i];
	}
	crc16 = CRC16_Check(buf, len + 4);
	buf[cnt++] = crc16 >> 8;
	buf[cnt++] = crc16 & 0xFF;
	buf[cnt++] = 0xFF;
	std::vector<uint8_t>buffer;
	for(short i=0 ;i< cnt;i++)
	{
		buffer.push_back(buf[i]);
	}
	Send(buffer); 
}

uint16_t SerialPort::CRC16_Check(const uint8_t* data, uint8_t len)
{
	uint16_t CRC16 = 0xFFFF;
	uint8_t state, i, j;
	for (i = 0; i < len; i++)
	{
		CRC16 ^= data[i];
		for (j = 0; j < 8; j++)
		{
			state = CRC16 & 0x01;
			CRC16 >>= 1;
			if (state)
			{
				CRC16 ^= 0xA001;
			}
		}  
	}
	return CRC16;
}
std::vector<int> SerialPort::save_data(uint8_t* data,int length)
{
	std::vector<int> vec1;
	//vec1.push_back(length);
	for(int i=0;i<length;i++)
	{
		int a;
		memcpy(&a,data+4*i,4);
		vec1.push_back(a);
	}
	return vec1;
}
void SerialPort::Data_Analysis(uint8_t cmd,uint8_t* datas, uint8_t len)
{
	switch (cmd)
	{
	case 0x00:
		this->data_length=len;
		// this->result_vec=this->save_data(datas,len/4);
		// uint8_t a;
		// memcpy(&a,datas,1);
		// this->lower_cmd=a;
        for(int i=0;i<len/4;i++)
        {
			// uint8_t a;
			// RCLCPP_INFO(this->logger_in,"接受到的数据是:%s",std::to_string(a).c_str());
			// memcpy(&a,datas+i,1);
			// this->lower_cmd=a;
			float a;
			memcpy(&a,datas+4*i,4);
			RCLCPP_INFO(this->logger_in,"接收到的数据是：%.2f",a);
        }
	
        std::cout<<std::endl;
		break;
	default:
		std::cerr << "Unknown command: " << cmd << std::endl;

		break;
	}

}

void SerialPort::Receive(uint8_t bytedata)
{
	
	static uint8_t step = 0; 
	static uint8_t cnt = 0, Buf[300], len, cmd, * data_ptr;
	static uint16_t   crc16;
   

	switch (step)
	{
	case 0: 
		if (bytedata == 0xA5)
		{
			step++;
			cnt = 0;
			Buf[cnt++] = bytedata;
		}
		break;
	case 1: 
		if (bytedata == 0x5A)
		{
			step++;
			Buf[cnt++] = bytedata;
		}
		else if (bytedata == 0xA5)
		{
			step = 1;
		}
		else
		{
			step = 0;
		}
		break;
	case 2: 
		step++;
		Buf[cnt++] = bytedata;
		len = bytedata;
		break;
	case 3: 
		step++;
		Buf[cnt++] = bytedata;
		cmd = bytedata;
		data_ptr = &Buf[cnt]; 
		if (len == 0)
			step++; 
		break;
	case 4: 
		Buf[cnt++] = bytedata;
		if (data_ptr + len == &Buf[cnt]) 
		{
			step++;
		}
		break;
	case 5: 
		step++;
		crc16 = bytedata;
		break;
	case 6: 
		crc16 <<= 8;
		crc16 += bytedata;
		if (crc16 == CRC16_Check(Buf, cnt)) 
		{
			step++;
		}
		else if (bytedata == 0xA5)
		{
			step = 1;
		}
		else
		{
			step = 0;
		}
		break;
	case 7:			     
		if (bytedata == 0xFF) 
		{
			//data=Data_Analysis(cmd, data_ptr);
            Data_Analysis(cmd,data_ptr,len);
			step = 0;
			
		}
		else if (bytedata == 0xA5)
		{
			step = 1;
		}
		else
		{
			step = 0;
		}
		break;
	default:
		step = 0;
		break; 
	}
}
void SerialPort::justice()
{
    return;

}
void SerialPort::save_data()
{

}
std::vector<int> SerialPort::got_data()
{
	std::vector<int> vec=this->result_vec;
	this->result_vec.clear();
	return vec;
}
int SerialPort::got_length()
{
	return this->data_length;
}
int SerialPort::got_baudRate()
{
	return this->baudRate;
}
int SerialPort::got_cmd()
{
	return this->lower_cmd;
}