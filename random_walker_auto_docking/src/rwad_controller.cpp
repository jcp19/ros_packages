#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kobuki_msgs/AutoDockingAction.h> 
#include <kobuki_msgs/AutoDockingActionGoal.h> 
// #include <kobuki_auto_docking/auto_docking_ros.hpp>
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
/* #define MINIMUM_LEVEL 40.0 */
/* #define MAXIMUM_LEVEL 70.0 */
double MINIMUM_LEVEL = 40.0;
double MAXIMUM_LEVEL = 70.0;
 
/* change the value if needed to match your robot's max battery value */
#define MAX_BATTERY 160
float battery_percentage = 0;

void batteryCallback(const kobuki_msgs::SensorState::ConstPtr& msg)
{
  uint8_t battery_level = msg->battery;
  battery_percentage = (battery_level * 100.0)/MAX_BATTERY;
  if(msg->charger != 0){
     mode = DOCKED;
  }
  ROS_INFO("battery_level: %f", battery_percentage);
}

/* TO-DO:
   call action for dockin
   add parameters to lowest value and maximum value battery (param server??)
   create a class for this node
   documentation
   put callbacks on action client 
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rwad_controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/mobile_base/sensors/core", 1, batteryCallback);
  //ros::Publisher pubVel =  n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher pubVel =  n.advertise<geometry_msgs::Twist>("input/rwad", 1000);
  ros::Publisher pubLed =  n.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);
  actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> ac("dock_drive_action", true);
  double min, max;
  if(ros::param::get(std::string("AUTODOCK_BATTERY_LEVEL"), min) && min > 0.0 && min < 100.0) 
      MINIMUM_LEVEL=min;
  if(ros::param::get(std::string("UNDOCK_BATTERY_LEVEL"), max) && max > 0.0 && max < 100.0 && max > MINIMUM_LEVEL) 
      MAXIMUM_LEVEL=max;
  ros::Rate r(4);

  while(ros::ok()){
    ros::spinOnce();
    if(battery_percentage <= MINIMUM_LEVEL && mode == RANDOM_WALK){
      mode = SEARCHING_DOCK;
      /* start service here */
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
      {
           // call the autodock action
           ROS_INFO("Waiting for action server to start."); 
           ac.waitForServer();
           kobuki_msgs::AutoDockingGoal goal; 
           ac.sendGoal(goal);
           bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

           if (finished_before_timeout)
           {
               actionlib::SimpleClientGoalState state = ac.getState();
               ROS_INFO("Action finished: %s",state.toString().c_str());
           }
           else
               ROS_INFO("Action did not finish before the time out.");
           
           break;
      }
      case RANDOM_WALK: 
           // no messages are sent, this way the messages from nodes with less priority will be read
           break;
    }
    r.sleep();
  }
  return 0;
}
