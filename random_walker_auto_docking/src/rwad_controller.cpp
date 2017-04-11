#include "ros/ros.h"
#include "kobuki_msgs/SensorState.h"
#include "kobuki_msgs/Led.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

/* possible robot states */
#define DOCKED 0
#define SEARCHING_DOCK 1
#define RANDOM_WALK 2
int mode = DOCKED; // its assumed that the robot starts in docked state

/* values for battery control */
#define MINIMUM_LEVEL 40.0
#define MAXIMUM_LEVEL 70.0
 
/* change the value if needed to match your robot's max battery value */
#define MAX_BATTERY 160
/* const uint8_t MAX_BATTERY = 160;*/
/* uint8_t battery_level; */
float battery_percentage;

void batteryCallback(const kobuki_msgs::SensorState::ConstPtr& msg)
{
  uint8_t battery_level = msg->battery;
  battery_percentage = (battery_level * 100.0)/MAX_BATTERY;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rwad_controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/mobile_base/sensors/core", 1, batteryCallback);
  ros::Publisher pubVel =  n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher pubLed =  n.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);
  ros::Rate r(4);
  while(true){
    ros::spinOnce();
    if(battery_percentage <= MINIMUM_LEVEL && mode == RANDOM_WALK){
      mode = SEARCHING_DOCK;
      kobuki_msgs::Led ledmsg;
      ledmsg.value = 2;
      pubLed.publish(ledmsg);
    } else if(battery_percentage >= MAXIMUM_LEVEL && mode == DOCKED){
      mode = RANDOM_WALK;
      kobuki_msgs::Led ledmsg;
      ledmsg.value = 0;
      pubLed.publish(ledmsg);
    }

    switch(mode){
      case DOCKED: 
      {
           // Empty message sent to override messages from nodes with less priority
           geometry_msgs::Twist vel;
           vel.linear.x = vel.linear.y = vel.linear.z = vel.angular.x = vel.angular.y = vel.angular.z = 0.0;
           pubVel.publish(vel); 
           break;
      }
      case SEARCHING_DOCK:
           // call the autodock action
           break;
      case RANDOM_WALK: 
           // no messages are sent, this way the messages from nodes with less priority will be read
           break;
    }
    r.sleep();
  }
  return 0;
}
