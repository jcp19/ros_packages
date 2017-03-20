#include "ros/ros.h"
#include "kobuki_msgs/Sound.h"
#include "wifi_info/wifi.h"

// WARNING: For this node to function properly, the time to execute the callback should be less than the 
// time between publishing in the wifi_info topic

// string essid;
ros::Publisher pub;

void beeperCallback(const wifi_info::wifi::ConstPtr& msg)
{
  // if(msg->essid == essid){ } // For now, it will beep for any network
  kobuki_msgs::Sound sound;
  sound.value = 3; // beeping sound

  int sl = msg->signal_level_dBm;
  int n_msgs = 9 - sl/-10;
  ros::Rate r(n_msgs);

  for(int i = 1; i < n_msgs; i++){
    pub.publish(sound);
    r.sleep();
  }
  
}

/* The first argument should be the essid of the network to track */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_beeper");
  ros::NodeHandle n;
  //essid = string(argv[1]);
  ros::Subscriber sub = n.subscribe("wifi", 1000, beeperCallback);
  pub = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1000);  

  ros::spin();

  return 0;
}

