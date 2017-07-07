#include "ros/ros.h"
#include "rwad2.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/Led.h"

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

void go_to_front_of_dock(int orientation = orientation_to_dock){

    // Not Implemented
}

void dock(){
    go_to_front_of_dock();
    // Not Implemented
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

void searching_dock_state(ros::Rate rate){

}

int main(int argc, char** argv){
    ros::init(argc, argv, "rwad2_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(8);

    // subscribes to read battery levels
    ros::Subscriber sub = n.subscribe(BATTERY_TOPIC, 1, battery_callback);

    ros::Publisher pubVel = n.advertise<geometry_msgs::Twist>("input/rwad", 10);
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

    ROS_INFO("Lower battery limit: %f\n", LOWER_LIMIT);
    ROS_INFO("Upper battery limit: %f\n", UPPER_LIMIT);

    // This loop goes through all the states of the automaton in each iteration
    while(ros::ok()){
        // Transition to docked state
        do_it(disableRandWalk);
        do_it(disableSafetyController);

        // Reached docked state
        docked_state(loop_rate);

        // Transition to walking state a.k.a undocking
        go_to_front_of_dock();
        do_it(enableRandWalk);
        do_it(enableSafetyController);

        // Reached docked state
        walking_state(loop_rate);

        // Reached dock searching state
        searching_dock_state(loop_rate);

        // Move the robot to the dock
        go_to_front_of_dock()
    }

    return 0;
}