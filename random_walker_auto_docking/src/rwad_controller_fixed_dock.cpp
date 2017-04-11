#include "ros/ros.h"
#include "kobuki_msgs/SensorState.h"
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
  ros::Publisher pub =  n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  while(true){
    ros::spinOnce();
    if(battery_percentage <= MINIMUM_LEVEL && mode = RANDOM_WALK){
      mode = SEARCHING_DOCK;
      // LIGAR LED
    } else if(battery_percentage >= MAXIMUM_LEVEL && mode = DOCKED){
      mode = RANDOM_WALK;
    }

    /* fazer coisas com base no modo */
    switch(mode){
      case DOCKED: /* mandar mensagens com vel a 0 para impedir de andar */
      case SEARCHING_DOCK:
      case RANDOM_WALK: /* nao mandar mensagens */
    }
  }
  return 0;
}
