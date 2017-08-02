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

// Odometry always starts at 0.0, so we need to track the differences over time.
float odomRobotX = 0.0;
float odomRobotY = 0.0;

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
    ros::spinOnce();
    ROS_INFO("Led color changed.");
}

bool go_to_front_of_dock(){
    bool success;
    ROS_INFO("Robot going to front of dock.");
    MoveBaseClient mbc("move_base", true);

    while(mbc.waitForServer(ros::Duration(2.0)))
        ROS_INFO("Waiting for move_base server.");

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "/map";
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
    ROS_INFO("Robot going to the dock.");
    AutoDockingClient adc("dock_drive_action", true);
    kobuki_msgs::AutoDockingGoal goal;

    while(adc.waitForServer(ros::Duration(2.0)))
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
    static int count = 0;
    int battery_level = msg->BATTERY_FIELD;
    robot_battery = battery_level * 100 / BATTERY_CAPACITY;
    count++;
    if(count == 30){
        count = 0;
        ROS_INFO("Battery level updated: %f", robot_battery);
    }
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg){
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    robotX += x - odomRobotX;
    robotY += y - odomRobotY;
    odomRobotX = x;
    odomRobotY = y;
}

void docked_state(ros::Rate rate, ros::Publisher disableSafetyController,
                  ros::Publisher disableRandWalk){
    while(robot_battery <= UPPER_LIMIT){
        do_it(disableSafetyController);
        do_it(disableRandWalk);
        ros::spinOnce();
        rate.sleep();
    }
}

float distance(float x1, float y1, float x2, float y2){
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

// steps back 1 meter from the dock
void undock(ros::Publisher rwadPub){
    geometry_msgs::Twist vel;
    vel.linear.x = -0.125;
    ros::Rate r(4);
    while(ros::ok() && distance(robotX, robotY, dockX, dockY) < 1){
        rwadPub.publish(vel);
        ros::spinOnce();
        r.sleep();
    }
}

void walking_state(ros::Rate rate, ros::Publisher enableSafetyController,
                   ros::Publisher enableRandWalk){
    while(ros::ok() && robot_battery >= LOWER_LIMIT){
        do_it(enableSafetyController);
        do_it(enableRandWalk);
        ros::spinOnce();
        rate.sleep();
    }
}

void load_battery_levels(){
    // Checks if the battery levels are defined in the param server and loads
    // them if they do
    double battery_param;
    if(ros::param::get(std::string("AUTODOCK_BATTERY_LEVEL"), battery_param) && battery_param > 0.0
       && battery_param < 100.0) {
        LOWER_LIMIT = battery_param;
    }
    if(ros::param::get(std::string("UNDOCK_BATTERY_LEVEL"), battery_param) && battery_param > 0.0 &&
            battery_param < 100.0 && battery_param > LOWER_LIMIT) {
        UPPER_LIMIT = battery_param;
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

    ros::Publisher led_topic = n.advertise<kobuki_msgs::Led>("rwad2_controller/led2", 10);
    ros::Publisher rwad_topic = n.advertise<geometry_msgs::Twist>("input/rwad",10);

    // controls the random walk activity
    auto enableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker_controller/enable", 1,true);
    auto disableRandWalk = n.advertise<std_msgs::Empty>("kobuki_random_walker_controller/disable", 1, true);

    // controls the safety controller activity
    auto enableSafetyController = n.advertise<std_msgs::Empty>("kobuki_safety_controller/enable", 1);
    auto disableSafetyController = n.advertise<std_msgs::Empty>("kobuki_safety_controller/disable", 1);

    do_it(disableSafetyController);
    do_it(disableRandWalk);
    ROS_INFO("Random Walker Disabled.");

    load_battery_levels();
    ROS_INFO("Lower battery limit: %f.", LOWER_LIMIT);
    ROS_INFO("Upper battery limit: %f.", UPPER_LIMIT);

    // This loop goes through all the states of the automaton in each iteration
    while(ros::ok()){
        // Robot in docked state
        change_color_led(led_topic, GREEN);
        ROS_INFO("Robot entering in docked state.");
        docked_state(loop_rate, enableSafetyController, enableRandWalk);
        if (!ros::ok()) return 0;

        // Transition to walking state a.k.a undocking
        change_color_led(led_topic, YELLOW);
        ROS_INFO("Robot undocking.");
        undock(rwad_topic);
        ROS_INFO("Robot undocked.");
        change_color_led(led_topic, DISABLE);
        if (!ros::ok()) return 0;

        // Robot in random walking state
        ROS_INFO("Robot entering random walk state.");
        walking_state(loop_rate, enableSafetyController, enableRandWalk);
        if (!ros::ok()) return 0;

        // The robot needs power, moves to the dock
        ROS_INFO("Robot leaving random walk state.");
        do_it(disableSafetyController);
        do_it(disableRandWalk);
        change_color_led(led_topic, RED);
        while(!go_to_front_of_dock())
            ROS_INFO("Robot didnt go to the front of the dock.");
        ROS_INFO("Robot docking.");
        dock();
    }

    return 0;
}
