#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kobuki_msgs/AutoDockingAction.h> 
#include <kobuki_msgs/AutoDockingActionGoal.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include "kobuki_msgs/SensorState.h"
#include "kobuki_msgs/Led.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

// dock coordinates
#define DOCK_X 1.0
#define DOCK_Y 1.0

/* possible robot states */
#define DOCKED 0
#define SEARCHING_DOCK 1
#define RANDOM_WALK 2
int mode = RANDOM_WALK; // its assumed that the robot starts in undocked state

/* values for battery control */
/* #define MINIMUM_LEVEL 40.0 */
/* #define MAXIMUM_LEVEL 70.0 */
double MINIMUM_LEVEL = 40.0;
double MAXIMUM_LEVEL = 70.0;
 
/* change the value if needed to match your robot's max battery value */
#define MAX_BATTERY 160
float battery_percentage = 0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void batteryCallback(const kobuki_msgs::SensorState::ConstPtr& msg)
{
  uint8_t battery_level = msg->battery;
  battery_percentage = (battery_level * 100.0)/MAX_BATTERY;
  /*if(msg->charger != 0){
     mode = DOCKED;
  }*/
}

/* TO-DO:
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
  ros::Publisher pubVel =  n.advertise<geometry_msgs::Twist>("input/rwad", 10);
  ros::Publisher pubLed =  n.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);
  ros::Publisher enableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker/enable", 1);
  ros::Publisher disableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker/disable", 1);
  ros::Publisher enableSafetyController = n.advertise<std_msgs::Empty>("kobuki_safety_controller/enable", 1);
  ros::Publisher disableSafetyController = n.advertise<std_msgs::Empty>("kobuki_safety_controller/disable", 1);
  actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> ac("dock_drive_action", true);
  MoveBaseClient move_base_client("move_base", true);

  // goal definition
  move_base_msgs::MoveBaseGoal g;
  g.target_pose.header.frame_id = "/map";
  //g.target_pose.header.stamp = ros::Time::now();
  g.target_pose.pose.position.x = DOCK_X;
  g.target_pose.pose.orientation.w = DOCK_Y;

  double min, max;
  if(ros::param::get(std::string("AUTODOCK_BATTERY_LEVEL"), min) && min > 0.0 && min < 100.0) 
      MINIMUM_LEVEL=min;
  if(ros::param::get(std::string("UNDOCK_BATTERY_LEVEL"), max) && max > 0.0 && max <= 100.0 && max > MINIMUM_LEVEL) 
      MAXIMUM_LEVEL=max;
  ros::Rate r(8);

  while(ros::ok()){
    ros::spinOnce();
    if(battery_percentage <= MINIMUM_LEVEL && mode == RANDOM_WALK){
      mode = SEARCHING_DOCK;
      /* start service here */
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

           while(!ac.waitForServer(ros::Duration(5.0))){
               ROS_INFO("Waiting for the move_base action server to come up");
           }
           g.target_pose.header.stamp = ros::Time::now();
           move_base_client.send(g);
           move_base_client.waitForResult();
           if(move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
               ROS_INFO("the robot moved to the base");
           else
               ROS_INFO("the robot failed to move to the base");

           ROS_INFO("Waiting for action server to start."); 
           ac.waitForServer();
           kobuki_msgs::AutoDockingGoal goal; 
           /* this section is not working properly */
           ac.sendGoal(goal);
           bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

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
