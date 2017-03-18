#include "ros/ros.h"
#include <sstream>
#include "wifi_info/wifi.h"
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>

std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

/* It is expected that the first argument(after remaps) is the name of the interface name */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_info");
  ros::NodeHandle n;

  // mudar tipo de mensagem
  ros::Publisher wifi_pub = n.advertise<wifi_info::wifi>("wifi", 1000);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    wifi_info::wifi msg;
    std::string cmd = "iwlist ";
    cmd.append(argv[1]);
    cmd.append(" scan | grep -v \"IE: Unknown\"");
    std::string networks = exec(cmd.c_str());
    msg.address = networks;


    //yy_scan_string(networks.c_str());
    //yylex();

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    wifi_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
