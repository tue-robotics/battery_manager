#include "ros/ros.h"
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <std_msgs/String.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>

using namespace std;

ros::Time time_init, time_current;
ros::Publisher pub;
ros::Publisher diag_pub;
ros::Subscriber analog_sub;
ros::Subscriber digital_sub;
int alarm_count = 0, max_alarms;
bool alarm_state = false;
double low_power_margin_ch1, high_power_margin_ch1, low_power_margin_ch2, high_power_margin_ch2, nominal_power, conversion_factor, alarm_period;
std_msgs::String alarm_msg;

void digitalCallback(const soem_beckhoff_drivers::DigitalMsg::ConstPtr& msg)
{
	vector<diagnostic_msgs::DiagnosticStatus> statuses;
	diagnostic_updater::DiagnosticStatusWrapper status;

	if (msg->values[0])
	{
		status.addf("Fuse RB", "broken");
		status.level = 0;
		status.message = "Fuse broken";
	}
	else
		status.addf("Fuse RB", "correct");
	if (msg->values[1])
	{
		status.addf("Fuse RF", "broken");
		status.level = 0;
		status.message = "Fuse broken";
	}
	else
		status.addf("Fuse RF", "correct");
	if (msg->values[2])
	{
		status.addf("Fuse LF", "broken");
		status.level = 0;
		status.message = "Fuse broken";
	}
	else
		status.addf("Fuse LF", "correct");
	if (msg->values[3])
	{
		status.addf("Fuse LB", "broken");
		status.level = 0;
		status.message = "Fuse broken";
	}
	else
		status.addf("Fuse LB", "correct");

	status.name = "Battery";

	status.level = 0;

	statuses.push_back(status);

	diagnostic_msgs::DiagnosticArray diag_msg;
	diag_msg.status = statuses;
	diag_msg.header.stamp = ros::Time::now();

	diag_pub.publish(diag_msg);
}
void analogCallback(const soem_beckhoff_drivers::AnalogMsg::ConstPtr& msg)
{
	vector<diagnostic_msgs::DiagnosticStatus> statuses;
	diagnostic_updater::DiagnosticStatusWrapper status;

	status.addf("Battery level", "%f", msg->values[0]*conversion_factor);

	status.name = "Battery";



	ROS_DEBUG("Battery value: %f", msg->values[0]);

	if (msg->values[0] < low_power_margin_ch1) {
		alarm_msg.data = string("Battery low");
		alarm_state = true;
		status.level = 1;
		status.message = "Battery low";

	} else if (msg->values[0] > high_power_margin_ch1) {
		alarm_msg.data = string("Power too high");
		alarm_state = true;
		status.level = 1;
		status.message = "Power too high";
	} else {
		alarm_msg.data = "";
		alarm_state = false;
		alarm_count = 0;	
		status.level = 0;
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


	statuses.push_back(status);

	diagnostic_msgs::DiagnosticArray diag_msg;
	diag_msg.status = statuses;
	diag_msg.header.stamp = ros::Time::now();

	diag_pub.publish(diag_msg);
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

	analog_sub = n.subscribe("/analog_in", 1000, analogCallback);
	digital_sub = n.subscribe("/digital_in", 1000, digitalCallback);
	pub = n.advertise<std_msgs::String>("/amigo_speak_up", 50);
	diag_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 50);

	time_init = ros::Time::now();	

	ros::Rate loop_rate(1.0);

	while(n.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
