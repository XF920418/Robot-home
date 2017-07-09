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

unsigned char Data_vx;
unsigned char Data_vy;
unsigned char Data_vth;

float vx;
float vy;
float vth;

double Last_vx;
double Last_vy;
double Last_vth;

//double odom_linear_scale_correction=1.0;	//用于矫正线速度
//double odom_angular_scale_correction=1.0;

unsigned char bufW[9],i=0;
unsigned char bufR[8];
unsigned char bufT[1];		//检测头

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
//    unsigned char bufR[8],i; 
    home_robot::serial_boost msg_encoder;    //创建发布速度信息的消息数据

    bufW[0] = 0xfe;
    bufW[1] = 0x01;	//标记位
    bufW[8]= 0xfd;

    serial_port sp(iosev, "/dev/ttyACM0");	//  USB转串口设备一般为/dev/ttyUSB*
    sp.set_option(serial_port::baud_rate(115200));//波特率
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));//流量控制
    sp.set_option(serial_port::parity(serial_port::parity::none));//奇偶校验
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));//停止位
    sp.set_option(serial_port::character_size(8));//字符大小
    
    while(1)
    {
      read(sp, buffer(bufT));
      if(bufT[0]==0xfe)
	break;
    }
    
   read(sp, buffer(bufR));	//从串口读取数据,直到读取指定字符的数据才会返回

    if(bufR[7] == 0xfd&&bufR[0]==0x01)//对串口数据进行头尾校验,读取底层返回的数据,注意这里的数据位个数
    {
      Data_vx  = bufR[2];	//16进制
      Data_vy = bufR[4];
      Data_vth = bufR[6];
      ROS_INFO("vx is %d",Data_vx);
      vx = (double)Data_vx/100;	//无需进制转换，看接收者的类型
      vy = (double)Data_vy/100;
      vth = (double)Data_vth/100;	
      if(bufR[1]==0&&abs(vx)>0)	//1正，0负
      {
	vx=-vx;
      }
      if(bufR[3]==0&&abs(vy)>0)
      {
	vy=-vy;
      }
      if(bufR[5]==0&&abs(vth)>0)
      {
	vth=-vth;
      }
      Last_vx  = vx;  //保持最后的速度状态
      Last_vy = vy;
      Last_vth = vth;   
    }
    else
    {
        vx  = Last_vx;
	vy = Last_vy;
        vth = Last_vth;
    }
    
    msg_encoder.linear_x  = vx;	//填充home_robot::serial_boost消息变量	
    msg_encoder.linear_y = vy;
    msg_encoder.angular_z = vth;
    chatter_pub.publish(msg_encoder);	//在"talker_boost_serial"话题下发布数据,发布从下位机获取的速度消息（编码器数据）
    
//    ROS_INFO("msg_encoder.linear_x  is %f",msg_encoder.linear_x);	//发布的从下位机获取的数据
//    ROS_INFO("msg_encoder.angular_z is %f",msg_encoder.angular_z);
    
    Twis_msg.Msg_line_ang.linear_x  = (float)msg.linear.x;		//订阅的cmd_vel话题的数据（可通过rostopic或者代码来发布该话题，速度值可以是负数）
    Twis_msg.Msg_line_ang.linear_y = (float)msg.linear.y;
    Twis_msg.Msg_line_ang.angular_z = (float)msg.angular.z;
    
    bufW[3] = (abs)(Twis_msg.Msg_line_ang.linear_x)*100;	//此处要注意负数进制转换的问题,重点在于buf[]是无符号字符，没有符号位	
    bufW[5] = (abs)(Twis_msg.Msg_line_ang.linear_y)*100;
    bufW[7] = (abs)(Twis_msg.Msg_line_ang.angular_z)*100;	 //无需进行进制转换，对于机器存储的都是二进制，需要用时直接用接收者来决定接受的进制数即可
    
    ROS_INFO("send is %d",bufW[3]);
    //方向
    if(Twis_msg.Msg_line_ang.linear_x<0)
    {
      bufW[2]=0;	// 后退
    }
    else 
    {
      bufW[2]=1;	//前进
    }
    
    if(Twis_msg.Msg_line_ang.linear_y<0)
    {
      bufW[4]=0;
    }
    else
    {
      bufW[4]=1;
    }
      
    if(Twis_msg.Msg_line_ang.angular_z<0)
    {
      bufW[6]=0;	//右转
    }
    else
    {
      bufW[6]=1;	//左转（右手定则）
    }
    
    ROS_INFO("msg.linear_x  is %f",Twis_msg.Msg_line_ang.linear_x);	//发给下位机的数据
    ROS_INFO("msg.linear_y  is %f",Twis_msg.Msg_line_ang.linear_y);
    ROS_INFO("msg.angular_z is %f",Twis_msg.Msg_line_ang.angular_z);
    write(sp, buffer(bufW, 9));    //向底层发送监听到的移动数据
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
