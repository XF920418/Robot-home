    #include <ros/ros.h>  
    #include <move_base_msgs/MoveBaseAction.h>  
    #include <actionlib/client/simple_action_client.h>  
      
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;  
      
    int main(int argc, char** argv)
    {  
      ros::init(argc, argv, "navigation_goals");  
      
      //tell the action client that we want to spin a thread by default  
      MoveBaseClient ac("move_base", true);  
      
      //wait for the action server to come up  
      while(!ac.waitForServer(ros::Duration(5.0)))
      {  
        ROS_INFO("Waiting for the move_base action server to come up");  
      }  
      
      move_base_msgs::MoveBaseGoal goal;  
   
      goal.target_pose.header.frame_id = "base_link";  
      goal.target_pose.header.stamp = ros::Time::now();  
      
      //position
      goal.target_pose.pose.position.x = 1.0;  
      goal.target_pose.pose.position.y = 0;
      goal.target_pose.pose.position.z = 0;
      
      //orientation
      goal.target_pose.pose.orientation.x = 0;
      goal.target_pose.pose.orientation.y = 0;
      goal.target_pose.pose.orientation.z = 0;
      goal.target_pose.pose.orientation.w = 1.0;  
      
      ROS_INFO("Sending goal");  
      ac.sendGoal(goal);  
      
      ac.waitForResult();  
      
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)  
        ROS_INFO("bingo ,reach the goal");  
      else  
        ROS_INFO("!!!!!fail, do not reach the goal");  
      
      return 0;  
    }  
