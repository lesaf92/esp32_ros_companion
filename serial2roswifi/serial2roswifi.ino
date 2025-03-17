#include <Arduino.h>
#include "WiFi.h"
#include <WiFiUdp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

// ESP32 IP
IPAddress ip(192, 168, 0, 101);
// rosserial host PC IP
IPAddress server(192, 168, 0, 104);
uint16_t serverPort = 11411; // coded in ArduinoTcpHardware.h
// WiFi credentials
const char *ssid = "LMI_2";
const char *password = "ea254@ea254";

// ros stuff
ros::NodeHandle  nh;
geometry_msgs::PoseStamped pose_msg;
ros::Publisher pose_pub("esp32_pose", &pose_msg);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setupWiFi()
{  
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());
}

// UDP for Time Sync (Optional, but recommended for ROS)
WiFiUDP udp;
unsigned int localPort = 2390; // Choose a free port
// 200.160.7.186 "a.st1.ntp.br"
IPAddress timeServerIP(200, 160, 7, 186); // NIST time server
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

unsigned long epoch_global;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to send NTP packet and receive time
unsigned long getTime() {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  udp.beginPacket(timeServerIP, 123); // NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();

  unsigned long startMillis = millis();
  while (udp.parsePacket() == 0) {
    if (millis() - startMillis > 1000) {
      Serial.println("NTP Timeout");
      return 0; // Return 0 if timeout.
    }
    delay(1);
  }

  udp.read(packetBuffer, NTP_PACKET_SIZE);

  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  const unsigned long seventyYears = 2208988800UL;
  unsigned long epoch = secsSince1900 - seventyYears;
  return epoch;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
    Serial.begin(115200);
    setupWiFi();

    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();

    // Another way to get IP
    Serial.print("ROS IP = ");
    Serial.println(nh.getHardware()->getLocalIP());

    // Start to be polite
    nh.advertise(pose_pub);

    //UDP setup for NTP
    udp.begin(localPort);

    // Get initial time sync
    epoch_global = getTime();
    if (epoch_global == 0) {
      Serial.println("NTP failed, time will be inaccurate");
    }
    else{
      Serial.println("NTP time sync successful");
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Read until newline

    // Parse the serial data (example: "x,y,z,qx,qy,qz,qw")
    float x, y, z, qx, qy, qz, qw;
    if (sscanf(data.c_str(), "%f,%f,%f,%f,%f,%f,%f", &x, &y, &z, &qx, &qy, &qz, &qw) == 7) {

      // Populate PoseStamped message
      if(epoch_global != 0){
        pose_msg.header.stamp.sec = getTime(); // Get current time from NTP
      }
      else{
        pose_msg.header.stamp.sec = millis() / 1000; // Get current time from microcontroller
      }
      pose_msg.header.stamp.nsec = (millis() % 1000) * 1000000; // Milliseconds to nanoseconds
      pose_msg.header.frame_id = "map"; // Change to your frame ID

      pose_msg.pose.position.x = x;
      pose_msg.pose.position.y = y;
      pose_msg.pose.position.z = z;

      pose_msg.pose.orientation.x = qx;
      pose_msg.pose.orientation.y = qy;
      pose_msg.pose.orientation.z = qz;
      pose_msg.pose.orientation.w = qw;

      // Publish the message
      pose_pub.publish(&pose_msg);
      nh.spinOnce(); // Handle ROS communication
    } else {
      Serial.println("Invalid serial data format");
    }
  }
  nh.spinOnce();
  delay(2); // Small delay to prevent flooding
}



