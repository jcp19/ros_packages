#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/Led.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <kobuki_msgs/AutoDockingActionGoal.h>
#include "rwad2.h"
#include <cmath>

/* led colors */
#define DISABLE 0
#define GREEN 1
#define YELLOW 2
#define RED 3

typedef int LedColor;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> AutoDockingClient;

// Dock Coordinates
float dockX = 1.41;
float dockY = -0.418;

// Robot Coordinates, matches dock coordinates at the beginning
float robotX = dockX;
float robotY = dockY;

// Coordinates 1 meter in front of dock
float frontX = 0.81;
float frontY = -0.124;

// Robot's state
float robot_battery = 0.0;

void do_it(ros::Publisher p){
    std_msgs::Empty e;
    p.publish(e);
}

void change_color_led(ros::Publisher led_topic, LedColor led_color){
    kobuki_msgs::Led msg;
    msg.value = led_color;
    led_topic.publish(msg);
}

bool go_to_front_of_dock(){
    bool success;
    MoveBaseClient mbc("move_base", true);

    while(!mbc.waitForServer(ros::Duration(2.0)))
        ROS_INFO("Waiting for move_base server.");

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = frontX;
    goal.target_pose.pose.position.y = frontY;

    mbc.sendGoal(goal);

    mbc.waitForResult();

    if(mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        success = true;
        ROS_INFO("Success: the robot went to the front of the dock.");
    }else{
        success = false;
        ROS_INFO("Insuccess: the robot didn't go to the front of the dock.");
    }

    return success;
}

bool dock(){
    /* docks the robot, assumes that the robot can see the dock */
    bool success = true;
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

    return success;
}

void battery_callback(const CALLBACK_ARG_TYPE::ConstPtr& msg){
    int battery_level = msg->BATTERY_FIELD;
    robot_battery = battery_level * 100 / BATTERY_CAPACITY;
    ROS_INFO("Battery level updated: %f", robot_battery);
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg){
    robotX = msg->pose.pose.position.x;
    robotY = msg->pose.pose.position.y;
}

void docked_state(ros::Rate rate){
    while(robot_battery <= UPPER_LIMIT){
        rate.sleep();
    }
}

float distance_squared(float x1, float y1, float x2, float y2){
    return pow(x1 - x2, 2) + pow(y1 - y2, 2);
}

// steps back 1 meter from the dock
void undock(ros::Publisher rwadPub){
    geometry_msgs::Twist vel;
    vel.linear.x = -2;
    ros::Rate r(4);
    while(distance_squared(robotX, robotY, dockX, dockY) < 1){
        rwadPub.publish(vel);
        r.sleep();
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
    ros::Subscriber subBattery = n.subscribe(BATTERY_TOPIC, 1, battery_callback);
    // subscribes to read robot Position
    ros::Subscriber subOdometry = n.subscribe("odom", 1, odometry_callback);

    ros::Publisher led_topic = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 10);
    ros::Publisher rwad_topic = n.advertise<geometry_msgs::Twist>("input/rwad",10);

    // controls the random walk activity
    auto enableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker_controller/enable", 1);
    auto disableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker_controller/disable", 1);

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
        // do_it(disableRandWalk);
        // do_it(disableSafetyController);

        // Robot in docked state
        change_color_led(led_topic, GREEN);
        docked_state(loop_rate);

        // Transition to walking state a.k.a undocking
        // go_to_front_of_dock();
        change_color_led(led_topic, YELLOW);
        undock(rwad_topic);
        change_color_led(led_topic, DISABLE);
        do_it(enableRandWalk);
        //do_it(enableSafetyController);
        //do_it(enableRandWalk);

        // Reached random walking state
        walking_state(loop_rate);
        do_it(disableRandWalk);

        // The robot needs power, moves to the dock
        bool in_front_of_dock = go_to_front_of_dock();
        if(in_front_of_dock){
            change_color_led(led_topic, RED);
            dock();
        }
    }

    return 0;
}
