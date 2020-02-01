#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>


ros::Publisher speech_pub;
ros::Subscriber percentage_sub;

double capacity;
double charge;
double percentage;


void batteryCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std:string location = "";
    if (msg.location == "hero2")
    {
        capacity = msg.capacity
        charge = msg.charge?
        percentage = (charge/capacity)*100
    }
}


int main(int argc, char **argv)
{
    ros::init(arc, argv, "Battery_manager_laptop")
    ros::NodeHandle gn;

    double middle = 50;
    double low = 20;
    double empty = 10;
    std::string message = "";

    battery_sub = gn.subscribe("battery", 1, batteryCallback);
    speech_pub = gn.advertise<std_msgs::String>("text_to_speech/input", 10);

    ros::Rate loop_rate(1.0)

    while(gn.ok())
    {
        // check battery status
        if (capacity < empty)
        {
            message = "The laptop battery is empty, I need power now!"
        }
        else if (capacity < low)
        {
            message = "The laptop battery is almost empty, give me power."
        }
        else if (capacity < middle)
        {
            message = "The laptop battery is halfway empty, charge me if possible. "
        }

        // publish voice message
        if (old_message != message && message != "")
        {
            std_msgs::String speech_msg;
            speech_msg.data = message;
            speech_pub.publish(speech_msg);
            old_message = message;
        }

    loop_rate.sleep();
    }
    return 0;
}

