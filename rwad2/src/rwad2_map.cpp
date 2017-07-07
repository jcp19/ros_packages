#include "ros/ros.h"
#include "rwad2.h"

// possibly not needed
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "kobuki_msgs/SensorState.h"
#include "kobuki_msgs/Led.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <kobuki_msgs/AutoDockingActionGoal.h>

// Dock Coordinates
float X;
float Y;

// Robot's state
STATE robot_state = DOCKED;
float robot_battery = 0.0;

void battery_callback(const CALLBACK_ARG_TYPE::ConstPtr& msg){
    int battery_level = msg->robot_battery;
    robot_battery = battery_level * 100 / BATTERY_CAPACITY;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "rwad2_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(8);

    // subscribes to read battery levels
    ros::Subscriber sub = n.subscribe(BATTERY_TOPIC, 1, battery_callback);

    ros::Publisher pubVel =  n.advertise<geometry_msgs::Twist>("input/rwad", 10);
    ros::Publisher pubLed =  n.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1);

    // controls the random walk activity
    ros::Publisher enableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker/enable", 1);
    ros::Publisher disableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker/disable", 1);

    // controls the safety controller activity
    ros::Publisher enableSafetyController = n.advertise<std_msgs::Empty>("kobuki_safety_controller/enable", 1);
    ros::Publisher disableSafetyController = n.advertise<std_msgs::Empty>("kobuki_safety_controller/disable", 1);

    // Checks if the battery levels are defined in the param server and loads
    // them if they do
    double battery_param;
    if(ros::param::get(std::string("rwad2/AUTODOCK_BATTERY_LEVEL"), battery_param) && battery_param > 0.0 && battery_param < 100.0) {
        LOWER_LIMIT = battery_param;
    }
    if(ros::param::get(std::string("rwad2/UNDOCK_BATTERY_LEVEL"), battery_param) && battery_param > 0.0 && battery_param < 100.0 && battery_param > MINIMUM_LEVEL) {
        UPPER_LIMIT = battery_param;
    }


    while(ros::ok()){
        docked_state(loop_rate);
        undocking();
        walking(loop_rate);
        search_dock(loop_rate);
    }

}