#include <ros/ros.h>
#include <soem_beckhoff_drivers/AnalogMsg.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "battery_test_client");
  ros::NodeHandle nh;
  
  ros::Publisher target_pub = nh.advertise<soem_beckhoff_drivers::AnalogMsg>("battery", 50);
  
  soem_beckhoff_drivers::AnalogMsg msg;
  
  msg.values.resize(2);
  
	msg.values[0] = 7.5;
	msg.values[1] = 0.0;

  ros::Rate rate(1.0);
  
  while (ros::ok()){
	ROS_INFO("Publish");  
    target_pub.publish(msg);
    rate.sleep();
  }
  
  return true;
}
