#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kobuki_msgs/AutoDockingAction.h> 
#include <kobuki_msgs/AutoDockingActionGoal.h> 
#include "kobuki_msgs/SensorState.h"
#include "kobuki_msgs/Led.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <iostream>

// use value 1 if you want to run it in a simulator or value 0 if not
#define SIMULATOR 1

#if SIMULATOR
  // when in simultaion, its assumed that the value of the battery is published in the /bokuki_simulation/battery topic
  #define BATTERY_TOPIC "/kobuki_simulation/battery"
  #define CALLBACK_ARG_TYPE std_msgs::UInt8 
  #define BATTERY_FIELD data 
#else
  #define BATTERY_TOPIC "/mobile_base/sensors/core"
  #define CALLBACK_ARG_TYPE kobuki_msgs::SensorState
  #define BATTERY_FIELD battery
#endif

// Possible robot states
#define DOCKED 0
#define SEARCHING_DOCK 1
#define RANDOM_WALK 2

// change the value if needed to match your robot's max battery value
#define MAX_BATTERY 160

// its assumed that the robot starts in undocked state
int mode = RANDOM_WALK; 

// values for battery control 
double MINIMUM_LEVEL = 40.0;
double MAXIMUM_LEVEL = 70.0;
 
float battery_percentage = 0;

void batteryCallback(const CALLBACK_ARG_TYPE::ConstPtr& msg)
{
  uint8_t battery_level = msg->BATTERY_FIELD;
  battery_percentage = (battery_level * 100.0)/MAX_BATTERY;
  /*if(msg->charger != 0){
     mode = DOCKED;
  }*/
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rwad_controller");
  ros::NodeHandle n;

  // topic to read battery level from
  ros::Subscriber sub = n.subscribe(BATTERY_TOPIC, 1, batteryCallback);

  ros::Publisher pubVel =  n.advertise<geometry_msgs::Twist>("input/rwad", 10);
  ros::Publisher pubLed =  n.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);

  // random walk control topics
  ros::Publisher enableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker/enable", 1);
  ros::Publisher disableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker/disable", 1);

  // safety controller control topics
  ros::Publisher enableSafetyController = n.advertise<std_msgs::Empty>("kobuki_safety_controller/enable", 1);
  ros::Publisher disableSafetyController = n.advertise<std_msgs::Empty>("kobuki_safety_controller/disable", 1);

  actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> ac("dock_drive_action", true);

  double min, max;
  if(ros::param::get(std::string("AUTODOCK_BATTERY_LEVEL"), min) && min > 0.0 && min < 100.0) {
      MINIMUM_LEVEL=min;
  }
  if(ros::param::get(std::string("UNDOCK_BATTERY_LEVEL"), max) && max > 0.0 && max <= 100.0 && max > MINIMUM_LEVEL) {
      MAXIMUM_LEVEL=max;
  }

  ROS_INFO("Minimum battery value is : %f", MINIMUM_LEVEL);
  ROS_INFO("Maximum battery value is : %f", MAXIMUM_LEVEL);

  ros::Rate r(8);

  while(ros::ok()){
    ros::spinOnce();
    if(battery_percentage <= MINIMUM_LEVEL && mode == RANDOM_WALK){
      mode = SEARCHING_DOCK;
      ROS_INFO("mode:%d, battery_level: %f", mode, battery_percentage);
      // start service here
      kobuki_msgs::Led ledmsg;
      ledmsg.value = 3;
      pubLed.publish(ledmsg);
    } else if(battery_percentage >= MAXIMUM_LEVEL && mode == DOCKED){
      std_msgs::Empty e;
      enableRandWalk.publish(e); 
      enableSafetyController.publish(e);
      mode = RANDOM_WALK;
      kobuki_msgs::Led ledmsg;
      ledmsg.value = 0;
      pubLed.publish(ledmsg);
    }

    ROS_INFO("mode:%d, battery_level: %f", mode, battery_percentage);
    switch(mode){
      case DOCKED: 
           break;
      case SEARCHING_DOCK:
      {
           // call the autodock action
           ROS_INFO("Waiting for action server to start."); 
           ac.waitForServer();
           kobuki_msgs::AutoDockingGoal goal; 
           // this section is not working properly
           ac.sendGoal(goal);
           bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

           if (finished_before_timeout)
           {
               actionlib::SimpleClientGoalState state = ac.getState();
               ROS_INFO("Action finished: %s",state.toString().c_str());
               std_msgs::Empty e;
               disableRandWalk.publish(e);
               disableSafetyController.publish(e);
               mode = DOCKED;
           }
           else
           {
               mode = RANDOM_WALK;
               ROS_INFO("Action did not finish before the time out.");
           }
      }break;
      case RANDOM_WALK: 
           // no messages are sent, this way the messages from nodes with less priority will be read
           break;
    }
    r.sleep();
  }
  return 0;
}
