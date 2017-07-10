#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/Led.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <kobuki_msgs/AutoDockingActionGoal.h>
#include "rwad2.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> AutoDockingClient;

// Dock Coordinates
float dockX = 1.41;
float dockY = -0.418;

// Coordinates 1 meter in front of dock
float frontX = 0.81;
float frontY = -0.124;
float orientation_to_dock = 0.0;

// Robot's state
float robot_battery = 0.0;

void do_it(ros::Publisher p){
    std_msgs::Empty e;
    p.publish(e);
}

bool go_to_front_of_dock(int orientation = orientation_to_dock){
    bool success;
    MoveBaseClient mbc("move_base", true);

    while(!mbc.waitForServer(ros::Duration(2.0)))
        ROS_INFO("Waiting for move_base server.");

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = frontX;
    goal.target_pose.pose.position.y = frontY;
    goal.target_pose.pose.orientation.w = orientation_to_dock;

    mbc.sendGoal(goal);

    mbc.waitForResult();

    if(mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        success = true;
        ROS_INFO("Success: the robot went to the front of the dock.");
    } else{
        success = false;
        ROS_INFO("Insuccess: the robot didn't go to the front of the dock.");
    }

    return success;
}

bool dock(){
    bool success;
    bool in_front_of_dock = success = go_to_front_of_dock();

    if(in_front_of_dock){
        AutoDockingClient adc("dock_drive_action", true);
        kobuki_msgs::AutoDockingGoal goal;

        while(!adc.waitForServer(ros::Duration(2.0)))
            ROS_INFO("Waiting for auto docking server.");

        adc.sendGoal(goal);
        bool finished_before_timeout = adc.waitForResult(ros::Duration(15.0));

        if(finished_before_timeout){
            ROS_INFO("Success: Robot docked.");
        } else {
            success = false;
            ROS_INFO("Insuccess: Robot didn't dock before the timeout.");
        }

    }

    return success;
}

void battery_callback(const CALLBACK_ARG_TYPE::ConstPtr& msg){
    int battery_level = msg->BATTERY_FIELD;
    robot_battery = battery_level * 100 / BATTERY_CAPACITY;
}

void docked_state(ros::Rate rate){
    while(robot_battery <= UPPER_LIMIT){
        rate.sleep();
    }
}

void walking_state(ros::Rate rate){
    while(robot_battery >= LOWER_LIMIT){
        rate.sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "rwad2_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(8);

    // subscribes to read battery levels
    ros::Subscriber sub = n.subscribe(BATTERY_TOPIC, 1, battery_callback);

    ros::Publisher pubLed = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1);

    // controls the random walk activity
    auto enableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker/enable", 1);
    auto disableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker/disable", 1);

    // controls the safety controller activity
    auto enableSafetyController = n.advertise<std_msgs::Empty>("kobuki_safety_controller/enable", 1);
    auto disableSafetyController = n.advertise<std_msgs::Empty>("kobuki_safety_controller/disable", 1);

    // Checks if the battery levels are defined in the param server and loads
    // them if they do
    double battery_param;
    if(ros::param::get(std::string("rwad2/AUTODOCK_BATTERY_LEVEL"), battery_param) && battery_param > 0.0
       && battery_param < 100.0) {
        LOWER_LIMIT = battery_param;
    }
    if(ros::param::get(std::string("rwad2/UNDOCK_BATTERY_LEVEL"), battery_param) && battery_param > 0.0 &&
            battery_param < 100.0 && battery_param > LOWER_LIMIT) {
        UPPER_LIMIT = battery_param;
    }

    ROS_INFO("Lower battery limit: %f.", LOWER_LIMIT);
    ROS_INFO("Upper battery limit: %f.", UPPER_LIMIT);

    // This loop goes through all the states of the automaton in each iteration
    while(ros::ok()){
        // Transition to docked state
        do_it(disableRandWalk);
        do_it(disableSafetyController);

        // Reached docked state
        docked_state(loop_rate);

        // Transition to walking state a.k.a undocking
        go_to_front_of_dock();
        do_it(enableSafetyController);
        do_it(enableRandWalk);

        // Reached random walking state
        walking_state(loop_rate);

        // The robot needs power, moves to the dock
        dock();
    }

    return 0;
}