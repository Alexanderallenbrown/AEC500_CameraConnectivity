#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

Servo driveservo;
Servo steerservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
// Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
// Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
int drivePin = 13;
int steerPin = 25;
int steerVal = 90;
int driveVal = 90;
/*
 * Resources:
 * https://www.upesy.com/blogs/tutorials/how-to-connect-wifi-acces-point-with-esp32?srsltid=AfmBOoqETd2LzbRVQhceepuJOgNTwvSER0QF5kaeFHQ7y1R6YbgEY0QD#
 * UDP EXAMPLE:
 * https://www.iotsharing.com/2017/06/how-to-use-udpip-with-arduino-esp32.html
 * unpacking UDP struct from python computer:
 * https://stackoverflow.com/questions/71957250/how-to-receive-the-struct-sent-over-udp-from-python-in-esp8266
 */
const char* ssid = "AEC500MechE2.4ghz";
const char* password = "4y0L8?&OBjg&";
// IP address to send UDP data to.
// it can be ip address of the server or 
// a network broadcast address
// here is broadcast address
const char * udpAddress = "192.168.1.200";//this is the address of the camera computer
const int udpPort = 44444;

//create UDP instance
WiFiUDP udp;

//this is how we define the format of the data coming in.
//in python, we use struct.pack('fff',*message) to indicate 3 floats in the packet.
//then, we'd send these data. You can change your data format as needed!
typedef struct commandMessage{
  float tcmd;
  float spdcmd;
  float steercmd;
};


void setup(){
  steerservo.setPeriodHertz(50);    // standard 50 hz servo
  pinMode(steerPin, OUTPUT);
  steerservo.attach(steerPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  pinMode(drivePin, OUTPUT);
  driveservo.attach(drivePin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  Serial.begin(115200);
  
  //Connect to the WiFi network
   WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop(){
  //data will be sent to server
  uint8_t buffer[50] = "requesting message";
  //This initializes udp and transfer buffer
  udp.beginPacket(udpAddress, udpPort);
  udp.write(buffer, 11);
  udp.endPacket();
  memset(buffer, 0, 50);
  uint8_t packet[1024];
  //processing incoming packet, must be called before reading the buffer
  int packetSize = udp.parsePacket();
  if (packetSize) {
//    Serial.printf("Received %d bytes\n", packetSize);
    if ((udp.read(packet, packetSize) > 0)) {
      commandMessage cmdmsg;                      //create a struct instance
      memcpy(&cmdmsg, packet, packetSize);  //copy packet array to the a struct

      driveVal = int(cmdmsg.spdcmd);
      steerVal = int(cmdmsg.steercmd);
      //Serial.printf("%f %f %f\n", 
       //             cmdmsg.tcmd, 
       //             cmdmsg.spdcmd, 
       //             cmdmsg.steercmd); // access the data in the struct
    }
  }
  else{
//    Serial.println("No Message received");
  }
  driveservo.write(driveVal);
  steerservo.write(steerVal);
//  Serial.printf("Steer %d \t Drive %d\n",steerVal,driveVal);
Serial.print("Steer: ");
Serial.print(steerVal);
Serial.print("\t Drive:");
Serial.println(driveVal);
  delay(10);
}
