#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "std_srvs/Empty.h"

void actionCallback(sensor_msgs::Joy msg)
{
  if (msg.buttons[1] == 1) {
    ros::NodeHandle nh;
  
    ros::ServiceClient process_client = nh.serviceClient<std_srvs::Empty>("process_view");
    std_srvs::Empty process_srv;
    
    ROS_INFO("Requesting to process view");
    if (process_client.call(process_srv)){
      ROS_INFO_STREAM("View processed!");
    } else {
      ROS_WARN_STREAM("View could not be processed!");
    }
  
    ros::WallDuration(1).sleep();      
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_listener");
  ros::NodeHandle nh;


  ros::Subscriber sub = nh.subscribe("joy", 1, actionCallback);

  ros::spin();

  return 0;
}

