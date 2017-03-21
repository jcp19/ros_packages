#include "ros/ros.h"
#include "kobuki_msgs/Sound.h"
#include "wifi_info/wifi.h"

// WARNING: For this node to function properly, the time to execute the callback should be less than the 
// time between publishing in the wifi_info topic

// string essid;
ros::Publisher pub;
ros::Rate * rates[10];
int n_msgs;
kobuki_msgs::Sound sound;

void beeperCallback(const wifi_info::wifi::ConstPtr& msg)
{
  // if(msg->essid == essid){ } // For now, it will beep for any network
  int sl = msg->signal_level_dBm;
  n_msgs = (9 - sl/-10)/2;
}

/* The first argument should be the essid of the network to track */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_beeper");
  ros::NodeHandle n;
  sound.value = 3; // beeping sound
  for(int i = 0; i < 10; i++) {
      rates[i] = new ros::Rate(i+1);
  }

  ros::Subscriber sub = n.subscribe("wifi", 1000, beeperCallback);
  pub = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1000);  
  
  while(true){
      ros::spinOnce();
      pub.publish(sound);
      rates[n_msgs]->sleep();
  }

  return 0;
}
