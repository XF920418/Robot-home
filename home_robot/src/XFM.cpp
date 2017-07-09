#include <iostream>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace boost::asio;

int main(int argc, char* argv[])
{
        io_service iosev;
	char str1[3];
	 int angle,sum=0;
	char bufR[200];		//读
	unsigned char bufW[6]={'R','E','S','E','T','\n'};
        //节点文件
        serial_port sp(iosev, "/dev/ttyUSB0");
        // 设置参数
        sp.set_option(serial_port::baud_rate(115200));
        sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp.set_option(serial_port::parity(serial_port::parity::none));
        sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp.set_option(serial_port::character_size(8));
	
	  ROS_INFO("start");
	  read(sp, buffer(bufR));
	  ROS_INFO("reading");
	  ROS_INFO("%s",bufR);
	  
	  char *str=strstr(bufR,"angle");
	  str1[0]=*(str+6);
	  str1[1]=*(str+7);
	  str1[2]=*(str+8);
	  cout<<endl<<str1[0]<<endl<<str1[1]<<endl<<str1[2]<<endl<<endl;
	  for(int i=0;i<3;i++)
	  {
	    if(str1[i]>='0'&&str1[i]<='9')
	      sum++;
	  }
	  cout<<"sum="<<sum<<endl;
	  if (sum==1)
	    angle=(str1[0]-'0');
	  else if(sum==2)
	    angle=(str1[0]-'0')*10+str1[1]-'0';
	  else if(sum==3)
	    angle=(str1[0]-'0')*100+(str1[1]-'0')*10+(str1[2]-'0');
	  
	  ROS_INFO("angle=%d\n",angle);
	  
	  cout<<"--------------------------------------------------------------"<<endl;

	  write(sp, buffer(bufW, 6)); 		//写入RESET
	  ROS_INFO("writing ok");

	iosev.run();
//	}
	return 0;
}
