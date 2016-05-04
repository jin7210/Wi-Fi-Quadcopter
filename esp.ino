#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <Servo.h>
/* Set these to your desired credentials. */
const char *ssid = "HUAWEI-6G32";
const char *password = "CmJrNxsh";

ESP8266WebServer server(80);

/* Go to http://192.168.4.1 in a web browser connected to this access point to see the control page. */

void handleRoot() {
  String rootPage="<h1>UAV Control Panel</h1><body>";
  rootPage+="<form action='/submit' method='put'>"; 
  
  rootPage+="<fieldset><legend>Throttle</legend>";
  rootPage+="<input type='radio' name='Throttle' value='#1500'> 1500";
  rootPage+="<input type='radio' name='Throttle' value='#1200'> 1200";
  rootPage+="<input type='radio' name='Throttle' value='#1000'> 1000";
  rootPage+="<input type='radio' name='Throttle' value='#1100' checked='checked'> 1100<br></fieldset>"; //defaults

  rootPage+="<fieldset><legend>Yaw</legend>";
  rootPage+="<input type='radio' name='Yaw' value='$1300'> 1300";
  rootPage+="<input type='radio' name='Yaw' value='$1700'> 1700";
  rootPage+="<input type='radio' name='Yaw' value='$1500' checked='checked'> 1500<br></fieldset>"; //defaults

  rootPage+="<fieldset><legend>Pitch</legend>";
  rootPage+="<input type='radio' name='Pitch' value='@1300'> 1300";
  rootPage+="<input type='radio' name='Pitch' value='@1700'> 1700";
  rootPage+="<input type='radio' name='Pitch' value='@1500' checked='checked'> 1500<br></fieldset>"; //defaults

  rootPage+="<fieldset><legend>Roll</legend>";
  rootPage+="<input type='radio' name='Roll' value='!1300'> 1300";
  rootPage+="<input type='radio' name='Roll' value='!1700'> 1700";
  rootPage+="<input type='radio' name='Roll' value='!1500' checked='checked'> 1500<br></fieldset>"; //defaults
  
  rootPage+="<input type='submit' value='submit'></form></body>";
 
	server.send(200, "text/html", rootPage);
}
void handleSubmit(){
  if(server.args()>0){
    for(int i=0; i<server.args(); i++){
      
            if(server.arg("Throttle")=="#1500"){ Serial.println("#1500");       }
              if(server.arg("Throttle")=="#1200"){ Serial.println("#1200");       }
                if(server.arg("Throttle")=="#1100"){ Serial.println("#1100");       }
                  if(server.arg("Throttle")=="#1000"){ Serial.println("#1000");       }

             if(server.arg("Yaw")=="$1500"){ Serial.println("$1500");       }
              if(server.arg("Yaw")=="$1300"){ Serial.println("$1300");       }
                if(server.arg("Yaw")=="$1700"){ Serial.println("$1700");       }

             if(server.arg("Pitch")=="@1500"){ Serial.println("@1500");       }
              if(server.arg("Pitch")=="@1300"){ Serial.println("@1300");       }
                if(server.arg("Pitch")=="@1700"){ Serial.println("@1700");       }

             if(server.arg("Roll")=="!1500"){ Serial.println("!1500");       }
              if(server.arg("Roll")=="!1300"){ Serial.println("!1300");       }
                if(server.arg("Roll")=="!1700"){ Serial.println("!1700");       }
            
        else{
          Serial.println("NA");
        }
      }
    }
      handleRoot();
}
void setup() { 
	Serial.begin(115200);
	WiFi.softAP(ssid, password);
	server.on("/", handleRoot);
  server.on("/submit",handleSubmit);
	server.begin();
  delay(10);
}
void loop() { 	server.handleClient();  }
