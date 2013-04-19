#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
//#include <pr2_msgs/PowerState.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>

using namespace std;

ros::Time time_init, time_current;
ros::Publisher power_pub;
ros::Publisher percentage_pub;
ros::Publisher diag_pub;
ros::Publisher speech_pub;
ros::Subscriber battery_sub;
ros::Subscriber fuse1_sub;
ros::Subscriber fuse2_sub;
ros::Subscriber fuse3_sub;
ros::Subscriber fuse4_sub;



bool fuse1, fuse2, fuse3, fuse4;
double voltage;
double conversion_factor;
string old_message;

void fuse1Callback(const std_msgs::Bool::ConstPtr& msg)
{
	fuse1=msg->data;
}

void fuse2Callback(const std_msgs::Bool::ConstPtr& msg)
{
	fuse2=msg->data;
}

void fuse3Callback(const std_msgs::Bool::ConstPtr& msg)
{
	fuse3=msg->data;
}
void fuse4Callback(const std_msgs::Bool::ConstPtr& msg)
{
	fuse4=msg->data;
}

void batteryCallback(const std_msgs::Float32::ConstPtr& msg)
{
	voltage=msg->data*conversion_factor;
	
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "battery_manager");
	ros::NodeHandle n;

	n.param<double> ("/battery_manager/conversion_factor", conversion_factor, 3.06);

	double max_voltage;
	double warn_voltage;
	double min_voltage;
	double full;
	double empty;
	bool batterywaslow;
	batterywaslow = false;
	n.param<double> ("/battery_manager/max_voltage", max_voltage, 30.0);
	n.param<double> ("/battery_manager/warn_voltage", warn_voltage, 23.0);
	n.param<double> ("/battery_manager/min_voltage", min_voltage, 21.0);
	n.param<double> ("/battery_manager/full_voltage", full, 27.0);
	n.param<double> ("/battery_manager/empty_voltage", empty, 22.0);


	//	pub = n.advertise<std_msgs::String>("/amigo_speak_up", 50);

	diag_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 50);


	fuse1_sub = n.subscribe("/fuse1", 1, fuse1Callback);
	fuse2_sub = n.subscribe("/fuse2", 1, fuse2Callback);
	fuse3_sub = n.subscribe("/fuse3", 1, fuse3Callback);
	fuse4_sub = n.subscribe("/fuse4", 1, fuse4Callback);
	battery_sub = n.subscribe("/battery_value", 1, batteryCallback);

//	power_pub = n.advertise<pr2_msgs::PowerState>("/power_state", 1);
	
	
	percentage_pub = n.advertise<std_msgs::Float32>("/battery_percentage", 1);
	diag_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
	speech_pub = n.advertise<std_msgs::String>("/amigo_speak_up", 10);


	time_init = ros::Time::now();	

	ros::Rate loop_rate(1.0);

	while(n.ok())
	{
		ros::spinOnce();

		ROS_DEBUG("Battery value: %f", voltage);
		ROS_DEBUG("Fuses: %i %i %i %i", fuse1, fuse2, fuse3, fuse4);

		// Calculate relative capacity
		double slope = 1/(full-empty);
		double offset = -slope*empty; 
		double capacity = max(0.0,min(100.0,(slope*voltage+offset)));
		int percentage = 100*capacity;
		percentage = std::min(100,percentage);

		// Create diagnostics
		diagnostic_updater::DiagnosticStatusWrapper status;
		status.message = "OK";
		status.level = 0;
		status.name = "Batteries";
		status.add("Battery level", voltage);

		string message = "";
		// Determine warning status
		if (voltage != 0.0)
		{
			if (voltage > max_voltage)
			{
				ROS_WARN("Voltage to high: %f V", voltage);
				status.message = "Voltage to high!";
				message = "My power is way to high, please do not fry my power circuits!";
				status.level = 1;
			}
			else if (voltage < min_voltage)
			{
				ROS_ERROR("Voltage seriously low: %f V", voltage);
				status.message = "Voltage seriously low!";
				message = "I need new batteries now! No, not later, NOW!";
				status.level = 2;
			}
			else if (voltage < warn_voltage)
			{
				ROS_WARN("Power low: %f V", voltage);
				status.message = "Voltage low!";
				message = "Please keep an eye on the batteries. Thank you.";
				status.level = 1;
				batterywaslow = true;
			}
			if (batterywaslow == true && (voltage > (warn_voltage*1.2))) {
				message = "Hah haah haah ha,    that tickles!";
				batterywaslow = false;
				status.level = 1;
			}
			
			
		}
		if (fuse1 || fuse2 || fuse3 || fuse4)
		{
			ROS_WARN("A fuse is broken!");
			status.message = "A fuse is broken!";
			message = "I am afraid that one of my fuses broke, can you please check it out";
			status.level = 1;
		}

		// Publish diagnostics
		vector<diagnostic_msgs::DiagnosticStatus> statuses;
		statuses.push_back(status);
		diagnostic_msgs::DiagnosticArray diag_msg;
		diag_msg.status = statuses;
		diag_msg.header.stamp = ros::Time::now();
		diag_pub.publish(diag_msg);

		// Publish voice message if new
		if (old_message != message)
		{
			std_msgs::String speech_msg;
			speech_msg.data = message;
			speech_pub.publish(speech_msg);
			old_message = message;
		}

		std_msgs::Float32 perc_msg;
		perc_msg.data = percentage;
		percentage_pub.publish(perc_msg);


		loop_rate.sleep();

	}
	return 0;
}
