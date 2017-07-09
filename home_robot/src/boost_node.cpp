#include <string>
#include <iostream>
#include <ros/ros.h>                           
#include <tf/transform_broadcaster.h>
#include <boost/asio.hpp>       
#include <boost/bind.hpp>
#include <math.h>
#include "std_msgs/String.h"           

using namespace std;
using namespace boost::asio;     

unsigned char buf[50];                      //定义字符串长度

int main(int argc, char** argv) 
{
    char a,b,c;
    ros::init(argc, argv, "boost_node");       //初始化节点
    ros::NodeHandle n;
    
   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);    

  ros::Rate loop_rate(10);

    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
    sp.set_option(serial_port::baud_rate(115200));   
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));
    
    ROS_INFO("command:");
    cin>>a>>b>>c;
    
    string s;
//    s=(string)argv;
    ROS_INFO("%c",a);
    ROS_INFO("%c",argv[2]);
    ROS_INFO("%c",argv[0]);
    

    while (ros::ok()) 
    {
      read (sp,buffer(buf));
      string str(&buf[0],&buf[22]);            //将数组转化为字符串
       std_msgs::String msg;
       std::stringstream ss;
       ss <<str;
     
      msg.data = ss.str();
     
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);   
    ros::spinOnce();
    loop_rate.sleep();
    }

    iosev.run(); 
    return 0;
}