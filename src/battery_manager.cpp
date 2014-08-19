#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>

using namespace std;

ros::Time time_init, time_current;
ros::Publisher power_pub;
ros::Publisher percentage_pub;
ros::Publisher diag_pub;
ros::Publisher speech_pub;
ros::Subscriber battery_sub;
std::vector<ros::Subscriber> fuse_subs;

bool fuse;
double voltage;
double conversion_factor;
string old_message;

void fuseCallback(const std_msgs::Bool::ConstPtr& msg)
{
  /// fuse variable is usually false, only copy data if something is wrong
  if (msg->data) {
    fuse = true;
  }
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
  bool belowticklevoltage;
  belowticklevoltage = false;
  n.param<double> ("/battery_manager/max_voltage", max_voltage, 30.0);
  n.param<double> ("/battery_manager/warn_voltage", warn_voltage, 23.0);
  n.param<double> ("/battery_manager/min_voltage", min_voltage, 21.0);
  n.param<double> ("/battery_manager/full_voltage", full, 27.0);
  n.param<double> ("/battery_manager/empty_voltage", empty, 22.0);
  double tickle_voltage;
  tickle_voltage = 25.0;

  diag_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 50);

  /// Subscribers
  std::string battery_sub_topic_name;
  n.param<std::string>("/battery_manager/battery_sub", battery_sub_topic_name, "/battery_value");
  battery_sub = n.subscribe(battery_sub_topic_name, 1, batteryCallback);

  XmlRpc::XmlRpcValue fuse_topic_names;
  if (!n.getParam("/battery_manager/fuse_subs", fuse_topic_names)) {
      ROS_ERROR("parameter /battery_manager/fuse_subs not found");
      return 0;
  }

  for (int32_t i = 0; i < fuse_topic_names.size(); ++i)
  {
    ROS_ASSERT(fuse_topic_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string topic_name = "/"+static_cast<std::string>(fuse_topic_names[i]);
    ros::Subscriber sub = n.subscribe(topic_name, 1, fuseCallback);
    fuse_subs.push_back(sub);
  }

  std::string battery_pub_topic_name;
  n.param<std::string>("/battery_manager/battery_pub", battery_pub_topic_name, "/battery_percentage");
  percentage_pub = n.advertise<std_msgs::Float32>(battery_pub_topic_name, 1);
  diag_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
  speech_pub = n.advertise<std_msgs::String>("/text_to_speech/input", 10);


  time_init = ros::Time::now();

  ros::Rate loop_rate(1.0);

  while(n.ok())
  {
    fuse = false;
    ros::spinOnce();

    ROS_DEBUG("Battery value: %f", voltage);
    ROS_DEBUG("Fuses: %i", fuse);

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
      }
      else if (voltage < tickle_voltage)
      {
        belowticklevoltage = true;
      }
      if (belowticklevoltage == true && (voltage > (tickle_voltage*1.05))) {

        int sentence=rand()%8;
        switch(sentence)
        {
                case 0:
                    message = "Hah haah haah ha,    that tickles!";
                    break;
                case 1:
                    message = "Yess yes yesss,    that feels sssso much better";
                    break;
                case 2:
                    message = "Come on,     That's more like it!";
                    break;
                case 3:
                    message = "Oh yeah,     this way I can go on for hours,,,,,, if you know what I mean";
                    break;
                case 4:
                    message = "Yeah baby,   You know exactly what I want";
                    break;
                case 5:
                    message = "I thought you never gave me some attention, but now I know.. I love you!";
                    break;
                case 6:
                    message = "What are you doing? I just feel energized";
                    break;
                case 7:
                    message = "Hah haah haah ha,    that tickles!";
                    break;
                case 8:
                    message = "Oh yeah,     this way I can go on for hours,,,,,, if you know what I mean";
                    break;
        }
        belowticklevoltage = false;
      }

    }
    if (fuse)
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
