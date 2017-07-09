/**
 * 
 * 
 **/
#include <ros/ros.h>
#include <home_robot/serial_boost.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iomanip>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>

//首先监听monitor_serial_node节点在talker_boost_serial话题下发布的线速度和角速度，然后发布tf转换和odom

class SubscribeAndPublish 
{ 
public: 
    double x;
    double y ;
    double th;

    double vx;
    double vy;
    double vth;

    ros::NodeHandle n; 
    ros::Publisher odom_pub;
    ros::Subscriber sub;
    ros::Time current_time, last_time;
    tf::TransformBroadcaster odom_broadcaster;
    
    SubscribeAndPublish() 
    { 
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);	//广播里程计数据,等待发布
        sub = n.subscribe("/talker_boost_serial", 10, &SubscribeAndPublish::chatterCallback,this); //监听发布的数据,订阅从下位机获取的速度
    } 
    
    void chatterCallback(const home_robot::serial_boost& msg) //msg类型如何确定
    { 
        current_time = ros::Time::now();
        vx = msg.linear_x;
	vy = msg.linear_y;	
        vth = msg.angular_z;
        ROS_INFO("cmd_vel linear.x  is %f",vx);	// 订阅的talker_boost_serial话题
	ROS_INFO("cmd_vel linear.y  is %f",vy);
        ROS_INFO("cmd_vel angular.z is %f",vth); 
        
    
	//更新里程计信息
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

	//将里程计的偏航角Yaw转换为四元数
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	//创建一个tf发布需要使用的TransformStamped类型消息,填充当前的时间戳、参考系id、子参考系id
        //注意两个参考系的id必须要是“odom”和“base_link”
	geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";	//参考系id
        odom_trans.child_frame_id = "base_link";	//子参考系id

	//填充里程计信息
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);	//该函数的目的？

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;		//stamp 是消息中与数据相关联的时间戳
        odom.header.frame_id = "odom";

	//填充机器人的位置
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;	//偏航角转化的四元数

	//填充机器人的速度信息
	//发布的是机器人本体的信息，所以参考系需要填"base_link"。
        odom.child_frame_id = "base_link";	
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        odom_pub.publish(odom);	//发布里程计数据
        last_time = current_time; 
    } 

};


int main(int argc, char **argv) 
{ 
    ros::init(argc, argv, "odometry_publisher");
    SubscribeAndPublish SAPObject; 
    SAPObject.current_time = ros::Time::now();
    SAPObject.last_time = ros::Time::now();
    ros::spin();
}
