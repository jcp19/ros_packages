%option c++

%{
#include "ros/ros.h"
#include "wifi_info/wifi.h"
#include <string>
#include <cstring>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <stdio.h>

wifi_info::wifi * info;

// return values:
#define NETWORK_READ 1
#define END 2

%}

%%
Cell	{ 
          ;
        }

Address:[[:space:]]([0-9A-F]+:)+[0-9A-F]+ {
			std::string s(yytext);
			std::string address = s.erase(0, strlen("Address: "));
			info->address = address;
		}
Channel:[0-9]+ {
		std::string s(yytext);
		std::string schannel = s.erase(0, strlen("Channel:"));
		int channel = std::stoi(schannel);
		info->channel = channel;
}

ESSID:\"[^\"]+\" {
		std::string s(yytext);
		std::string essid = s.erase(0, strlen("ESSID:\""));
		essid.erase(essid.size()-1, essid.size());
		info->essid = essid;
		return NETWORK_READ;
}

Quality=[0-9]+\/[0-9]+ {
		std::string q(yytext);
		q.erase(0, strlen("Quality="));
		info->quality = q;
	}

Signal[[:space:]]level=-[0-9]+ {
		std::string sl(yytext);
		sl.erase(0, strlen("Signal level="));
		info->signal_level_dBm = std::stoi(sl);
}	


<<EOF>> { return END; }
.|\n {;}
%%

extern "C" {
  int yywrap();
}

int yyFlexLexer::yywrap() {
  return ::yywrap();
}

extern "C" {
  YY_BUFFER_STATE yy_scan_string(const char *);
}

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
/* If the 3rd argument is provided, it is considered to be te name of the network to analyse - only data of that network will 
be printed */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_info");
  if(argc < 2){
	std::cerr << "You need to pass the interface name" << std::endl;
	return 1;
  }
  ros::NodeHandle n;

  ros::Publisher wifi_pub = n.advertise<wifi_info::wifi>("wifi", 1000);

  ros::Rate loop_rate(1);

  std::string cmd = "iwlist ";
  cmd.append(argv[1]);
  cmd.append(" scan | grep -v \"IE: Unknown\"");
  char * c = cmd.c_str();
  
  while (ros::ok())
  {
    std::string networks = exec(c);
    std::istringstream ss(networks);

    yyFlexLexer lex;
    lex.switch_streams(&ss);
    info = new wifi_info::wifi();
    if(argc == 2){
    	while(lex.yylex() == NETWORK_READ){
      	    // create a new info to ensure that no old data field is used in a different network if some field is ommited 
            wifi_pub.publish(*info); 
	    delete(info);
	    info = new wifi_info::wifi();
        }
    } else {
	std::string essid = std::string(argv[2]);
	while(lex.yylex() == NETWORK_READ){
            if(info->essid == essid){
                wifi_pub.publish(*info); 
            }
	    delete(info);
	    info = new wifi_info::wifi();
        }
    }
    delete(info);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
