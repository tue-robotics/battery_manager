#include "ros/ros.h"
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <std_msgs/String.h>

using namespace std;

ros::Time time_init, time_current;
ros::Publisher pub;
ros::Subscriber sub;
int alarm_count = 0, max_alarms;
bool alarm_state = false;
double low_power_margin_ch1, high_power_margin_ch1, low_power_margin_ch2, high_power_margin_ch2, nominal_power, conversion_factor, alarm_period;
std_msgs::String alarm_msg;


void batteryCallback(const soem_beckhoff_drivers::AnalogMsg::ConstPtr& msg)
{
	ROS_DEBUG("Battery value: %f", msg->values[0]);
	
	if (msg->values[0] < low_power_margin_ch1) {
		alarm_msg.data = string("Power too low on channel one");
		alarm_state = true;
	} else if (msg->values[0] > high_power_margin_ch1) {
		alarm_msg.data = string("Power too high on channel one");
		alarm_state = true;
	} else {
		alarm_msg.data = "";
		alarm_state = false;
		alarm_count = 0;	
	}
	
	if (alarm_state) {
		if (alarm_count == 0) {
			time_init = ros::Time::now();
		}
		if (alarm_count < max_alarms) {
			pub.publish(alarm_msg);
			alarm_count++;
		}
	}
	
	time_current = ros::Time::now();	
	
	if (alarm_state && (time_current.toSec()-time_init.toSec() > alarm_period)) {
		alarm_count = 0;
		alarm_state = false;
		alarm_msg.data = "";
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "battery_manager");
	ros::NodeHandle n;

	n.param<double> ("/battery_manager/low_power_margin_ch1", low_power_margin_ch1, 7.0);
	n.param<double> ("/battery_manager/high_power_margin_ch1", high_power_margin_ch1, 9.0);
	n.param<double> ("/battery_manager/low_power_margin_ch2", low_power_margin_ch2, 7.0);
	n.param<double> ("/battery_manager/high_power_margin_ch2", high_power_margin_ch2, 9.0);
	n.param<double> ("/battery_manager/nominal_power", nominal_power, 7.5);
	n.param<double> ("/battery_manager/conversion_factor", conversion_factor, 3.0);
	n.param<double> ("/battery_manager/alarm_period", alarm_period, 10.0);
	n.param<int> ("/battery_manager/max_alarms", max_alarms, 3);
	
	sub = n.subscribe("/battery", 1000, batteryCallback);
	pub = n.advertise<std_msgs::String>("/amigo_speak_up", 50);
	
	time_init = ros::Time::now();	
	
	ros::Rate loop_rate(1.0);

	while(n.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
