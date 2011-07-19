#include <ros/ros.h>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "battery_test_client");
  ros::NodeHandle nh;
  
  ros::Publisher atarget_pub = nh.advertise<soem_beckhoff_drivers::AnalogMsg>("analog_in", 50);
  ros::Publisher dtarget_pub = nh.advertise<soem_beckhoff_drivers::DigitalMsg>("digital_in", 50);
  
  soem_beckhoff_drivers::AnalogMsg amsg;
  soem_beckhoff_drivers::DigitalMsg dmsg;
  
  amsg.values.resize(2);
  dmsg.values.resize(8);
  
	amsg.values[0] = 6.5;
	amsg.values[1] = 0.0;

	dmsg.values[1] = true;

  ros::Rate rate(1.0);
  
  while (ros::ok()){
	ROS_INFO("Publish");  
    atarget_pub.publish(amsg);
    dtarget_pub.publish(dmsg);
    rate.sleep();
  }
  
  return true;
}
