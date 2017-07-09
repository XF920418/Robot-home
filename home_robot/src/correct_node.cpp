#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>

using namespace std;
using namespace boost::asio;

#define BYTE0(pointer) (*((char*)(&pointer)+0));
#define width_robot 0.31
#define ticks_per_meter 400000
#define MS_RADMIN 233

ros::Time current_time, last_time;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
unsigned char buf[6] = {0xff,0x00,0x00,0x00,0x00,0xfe};
unsigned char buf1[10];
float Data_vx;
float Data_vth;

int HerdWore_vx;
int HerdWore_vth;
  ros::NodeHandle n;
geometry_msgs::Quaternion odom_quat;
 tf::TransformBroadcaster odom_broadcaster;
void cmd_velCallback(const geometry_msgs::Twist &twist_aux)  //具体数据处理在这个毁掉函数里，我的pid单位是转/分钟 所以存在一个常量。
{
   ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
   double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;
        x += delta_x;
        y += delta_y;
        th += delta_th;
        
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        
        odom_broadcaster.sendTransform(odom_trans);
        
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        
        odom_pub.publish(odom);
        last_time = current_time;
}

int main(int argc, char** argv)
{
    unsigned char buf1[10],i;

    io_service iosev;
        serial_port sp(iosev, "/dev/ttyUSB0");
        sp.set_option(serial_port::baud_rate(115200));
        sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp.set_option(serial_port::parity(serial_port::parity::none));
        sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp.set_option(serial_port::character_size(8));

    ros::init(argc, argv, "base_controller");
    
   
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, cmd_velCallback);
    
    
    current_time = ros::Time::now();
     last_time = ros::Time::now();
    while(ros::ok())
    {
        ros::spinOnce();  
        write(sp, buffer(buf, 6));  //write the speed for cmd_val             
        current_time = ros::Time::now();
        read(sp, buffer(buf1));
        
         if(buf1[0] == 0xfd && buf1[5] == 0xfe)//对串口数据进行头尾校验,读取底层返回的数据
    {
      Data_vx  = buf1[1];	//16进制
      Data_vth = buf1[2];
      vx = (double)Data_vx/100;
      vth = (double)Data_vth/100;	
      if(buf1[3]==0&&vx!=0)
	vx=-vx;
      if(buf1[4]==0&&vth!=0)
	vth=-vth;
      
    }
           
    }
    iosev.run(); 
}