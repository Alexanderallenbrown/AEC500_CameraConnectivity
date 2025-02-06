#include <WiFi.h>
#include <WiFiUdp.h>
/*
 * Resource:
 * https://www.upesy.com/blogs/tutorials/how-to-connect-wifi-acces-point-with-esp32?srsltid=AfmBOoqETd2LzbRVQhceepuJOgNTwvSER0QF5kaeFHQ7y1R6YbgEY0QD#
 * UDP EXAMPLE:
 * https://www.iotsharing.com/2017/06/how-to-use-udpip-with-arduino-esp32.html
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

void setup(){
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
  uint8_t buffer[50] = "hello world";
  //This initializes udp and transfer buffer
  udp.beginPacket(udpAddress, udpPort);
  udp.write(buffer, 11);
  udp.endPacket();
  memset(buffer, 0, 50);
  //processing incoming packet, must be called before reading the buffer
  udp.parsePacket();
  //receive response from server, it will be HELLO WORLD
  if(udp.read(buffer, 50) > 0){
    Serial.print("Server to client: ");
    Serial.println((char *)buffer);
  }
  //Wait for 1 second
  delay(1000);
}
