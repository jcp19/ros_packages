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
		//info->quality = q;
	}

Signal[[:space:]]level=-[0-9]+ {
		std::string sl(yytext);
		//info->signal_level_dBm = sl;
}	


<<EOF>> { return END; }
.|\n {;}
%%

/* por  */

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
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_info");
  if(argc < 2){
	std::cerr << "You need to pass the interface name";
	return 1;
  }
  ros::NodeHandle n;

  ros::Publisher wifi_pub = n.advertise<wifi_info::wifi>("wifi", 1000);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    std::string cmd = "iwlist ";
    cmd.append(argv[1]);
    cmd.append(" scan | grep -v \"IE: Unknown\"");
    std::string networks = exec(cmd.c_str());

    std::istringstream ss(networks);

    yyFlexLexer lex;
    lex.switch_streams(&ss);
    info = new wifi_info::wifi();
    while(lex.yylex() == NETWORK_READ){
        wifi_pub.publish(*info);
	//clean shit
	info = new wifi_info::wifi();
    }
    //clean shit

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
