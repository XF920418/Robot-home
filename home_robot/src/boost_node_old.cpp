#include <iostream>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace boost::asio;

int main(int argc, char* argv[])
{
        io_service iosev;
	float vx,vy,vth;
	unsigned char Data_vx,Data_vy,Data_vth;
	unsigned char bufR[8];		//读
	unsigned char bufW[9];	//写
	unsigned char bufT[1];		//检测头
	double Last_vx,Last_vy,Last_vth;
        //节点文件
        serial_port sp(iosev, "/dev/ttyACM0");
        // 设置参数
        sp.set_option(serial_port::baud_rate(115200));
        sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp.set_option(serial_port::parity(serial_port::parity::none));
        sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp.set_option(serial_port::character_size(8));
	
	while(1)
	{
	  while(1)
	  {
	    read(sp, buffer(bufT));
	    if(bufT[0]!=0xfe)
	      continue;
	    else break;
	  }
	  read(sp, buffer(bufR));
	  
	  for(int i=0;i<=7;i++)
	    printf("%d\n",bufR[i]);
	if(bufR[7] == 0xfd&&bufR[0]==0x01)//对串口数据进行头尾校验,读取底层返回的数据,注意这里的数据位个数
	{
	  ROS_INFO("aaaa");
	  Data_vx  = bufR[2];	//16进制
	  Data_vy = bufR[4];
	  Data_vth = bufR[6];
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
	ROS_INFO("vx  is %f",vx);	
	ROS_INFO("vy  is %f",vy);	
	ROS_INFO("vth is %f",vth);
	
	iosev.run();
	}
	return 0;
}
