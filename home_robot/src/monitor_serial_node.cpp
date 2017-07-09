/**
 * 
 *
 * 
 * */
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <home_robot/serial_boost.h>
#include <iomanip>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>

using namespace std;
using namespace boost::asio;


unsigned char Data_vth;
unsigned char Data_vx;
unsigned char Data_vy;

float vx;
float vy;
float vth;

//int HerdWore_vx;
//int HerdWore_vth;

double Last_vx;
double Last_vy;
double Last_vth;

//double odom_linear_scale_correction=1.0;	//用于矫正线速度
//double odom_angular_scale_correction=1.0;

unsigned char buf[6],i=0;

//union(联合) 中所有的数据成员共用一个空间，配置一个足够大的空间用来容纳最大长度的数据成员,
//同一时间只能储存其中一个数据成员，所有的数据成员具有相同的起始地址，多用来压缩空间.
union _data_{
struct _struct_float{
    float linear_x;
    float linear_y;
    float angular_z;
    }Msg_line_ang;
}Twis_msg;

ros::Publisher chatter_pub;

void Recevice_Serial_Data(const geometry_msgs::Twist& msg)
{
    io_service iosev;  //boost串口端口配置
    unsigned char buf1[6],i; 
    home_robot::serial_boost msg_encoder;    //创建发布速度信息的消息数据

    buf[0] = 0xfb;
    buf[5] = 0xfe;

    serial_port sp(iosev, "/dev/ttyUSB0");	//  USB转串口设备一般为/dev/ttyUSB*
    sp.set_option(serial_port::baud_rate(115200));//波特率
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));//流量控制
    sp.set_option(serial_port::parity(serial_port::parity::none));//奇偶校验
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));//停止位
    sp.set_option(serial_port::character_size(8));//字符大小
    
   read(sp, buffer(buf1));	//从串口读取数据,直到读取指定字符的数据才会返回


    ROS_INFO("msg.linear_x  is %f",Twis_msg.Msg_line_ang.linear_x);	//发给下位机的数据
    ROS_INFO("msg.angular_z is %f",Twis_msg.Msg_line_ang.angular_z);
    write(sp, buffer(buf, 6));    //向底层发送监听到的移动数据
    iosev.run();		//函数会自动的调用异步操作函数指定到回调函数
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"listener_cmd_vel");//节点名称
    ros::NodeHandle node_Handle;
    chatter_pub = node_Handle.advertise<home_robot::serial_boost>("talker_boost_serial", 10);//发布话题
    ros::Subscriber sub = node_Handle.subscribe("/cmd_vel",10,&Recevice_Serial_Data);	//监听/cmd_vel话题下的移动数据
    ros::spin();//没有spin()就不会调用回调函数，该函数进入自循环,尽可能快的调用消息回调函数。spin先启动，有消息到来时，才会调用回调函数
    return 0;
}
