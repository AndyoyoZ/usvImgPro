//serial_ros_node
//by:Andyoyo@swust
//data:2018.12.25
#include <string>
#include <ros/ros.h>                           // 包含ROS的头文件
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
#include "std_msgs/String.h"              //ros定义的String数据类型

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");      //定义传输的串口
unsigned char buf[24];                      //定义字符串长度
int lineSpeed=0;
int angleVelocity=0;
unsigned char floatvx[2];
unsigned char floatvw[2];

void long_char(int cmdvel)
{       
	floatvx[0]=cmdvel>>8;
	floatvx[1]=cmdvel;
}
void long_char1(int cmdvel)
{       
	floatvw[0]=cmdvel>>8;
	floatvw[1]=cmdvel;
}

//计算CRC值
unsigned short count_CRC(unsigned char *addr, int num)
{
	unsigned short CRC = 0xFFFF;
	int i;
	while (num--)
	{
		CRC ^= *addr++;
		for (i = 0; i < 8; i++)
		{
		if (CRC & 1)
		{
		CRC >>= 1;
		CRC ^= 0xA001;
		}
		else
		{
		CRC >>= 1;
		}
		}
	}
	return CRC;
}

unsigned char cmd_buff[15];
void set_cmdnum()
{      
	unsigned short crcs;
	cmd_buff[0]=0xaa;
	cmd_buff[1]=0x55;
	cmd_buff[2]=0x60;       //地址
	cmd_buff[3]=floatvx[0];
	cmd_buff[4]=floatvx[1];
	cmd_buff[5]=floatvw[0];
	cmd_buff[6]=floatvw[1];
	crcs=count_CRC(cmd_buff, 7);
	cmd_buff[7]=(crcs&0xff00)>>8;
	cmd_buff[8]=(crcs&0x00ff);
	cmd_buff[9]=0xdd;
}

void write_all(int vx, int vw)   
{
  long_char(vx);
  long_char1(vw);
  set_cmdnum();
  boost::asio::write(sp,buffer(cmd_buff));   //写入下位机数据
}

void sendVelocityCB(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
   
    lineSpeed = 1000*vel_msg->linear.x;    //0-200
    angleVelocity = 200*vel_msg->angular.z;//0-200
    usleep(10000);  //10ms
    ROS_INFO("send Velocity...%d, %d",lineSpeed,angleVelocity);
    write_all(lineSpeed,angleVelocity);
    //write_all(1,2);
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "serial_ros");       //初始化节点
    ros::NodeHandle n1;
    ros::Subscriber velocity_sub = n1.subscribe<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1,sendVelocityCB);

    ros::Rate loop_rate(10);
 

    //串口设置
    ROS_INFO("设置串口");
    sp.set_option(serial_port::baud_rate(9600));  //波特率   9600  
    sp.set_option(serial_port::flow_control());   //流量控制 无
    sp.set_option(serial_port::parity());         //奇偶校验 无
    sp.set_option(serial_port::stop_bits());      //停止位   1
    sp.set_option(serial_port::character_size(8));//字符大小 8

    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    iosev.run(); 
    return 0;
}