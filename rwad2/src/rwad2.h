#ifndef RWAD2_H
#define RWAD2_H

#include "kobuki_msgs/SensorState.h"
#include "std_msgs/UInt8.h"
// Robot states
enum STATE {DOCKED, WALKING, SEARCHING_DOCK};

// Battery levels in which state transitions will occur
float LOWER_LIMIT = 80.0f;
float UPPER_LIMIT = 85.0f;

const float BATTERY_CAPACITY = 160.0;

#ifdef SIMULATION
  // when in simultaion, its assumed that the value of the battery
  // is published in the /bokuki_simulation/battery topic
  #define BATTERY_TOPIC "/kobuki_simulation/battery"
  #define CALLBACK_ARG_TYPE std_msgs::UInt8
  #define BATTERY_FIELD data
#else
  #define BATTERY_TOPIC "/mobile_base/sensors/core"
  #define CALLBACK_ARG_TYPE kobuki_msgs::SensorState
  #define BATTERY_FIELD battery
#endif //SIMULATION

#endif //RWAD2_H
