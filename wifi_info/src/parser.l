%option c++
%{
#include "wifi_info/wifi.h"
#include <string>
#include <cstring>
wifi_info::wifi * info = nullptr;

// return values:
#define NETWORK_READ 1

%}

%%
Cell	{ 
          wifi_info::wifi * tp = info;
          info = new wifi_info::wifi();
        }

"Address: ([0-9]+:)+[0-9]+" {
			using namespace std;
			string s(yytext);
			string address = s.erase(0, strlen("Address: "));
			(*info).address = address;
		}
"Channel:[0-9]+" {
		using namespace std;
		string s(yytext);
		string schannel = s.erase(0, strlen("Channel:"));
		int channel = std::stoi(schannel);
		(*info).channel = channel;
}

"ESSID:\"[^\"]+\"" {
		using namespace std;
		string s(yytext);
		string ssid = s.erase(0, strlen("ESSID:\""));
		ssid.erase(ssid.size()-2, ssid.size());
		(*info).essid = ssid;
		return NETWORK_READ;
}

%%

extern "C" {
  int yywrap();
}

int yyFlexLexer::yywrap() {
  return ::yywrap();
}
