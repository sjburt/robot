// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <ctime>
#include <cmath>
#include <Adafruit_GPS.h>
#include <ros.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>

HardwareSerial mySerial = HardwareSerial();
Adafruit_GPS GPS(&mySerial);
ros::NodeHandle nh;

std_msgs::Int32 num_sats_msg;
sensor_msgs::NavSatFix nav_msg;
ros::Publisher nav("gps", &nav_msg);
ros::Publisher num_sats("num_sats", &num_sats_msg);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(nav);
  nh.advertise(num_sats);
  delay(2000);
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  num_sats_msg.data = 0;
  nav_msg.header.frame_id = "my_frame";
  nav_msg.header.seq = 0;
  nav_msg.status.status = -1;
  nav_msg.status.service = 1;

}

void serialEvent1() {
  while (mySerial.available()) {
     GPS.read();
     static uint8_t p;
     digitalWrite(13, p=!p);
   }
}

void loop()                     // run over and over again
{

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    struct tm timeinfo;
    timeinfo.tm_year = GPS.year + 100;
    timeinfo.tm_mon = GPS.month - 1;
    timeinfo.tm_mday = GPS.day;
    timeinfo.tm_hour = GPS.hour;
    timeinfo.tm_min = GPS.minute;
    timeinfo.tm_sec = GPS.seconds;
    time_t sec = mktime(&timeinfo);

    nav_msg.header.stamp.sec = sec;
    nav_msg.header.stamp.nsec = GPS.milliseconds * 1000000;
    nav_msg.header.seq++;
    if (GPS.fix) {
      nav_msg.status.status = 0;
      nav_msg.latitude = GPS.latitudeDegrees;
      nav_msg.longitude = GPS.longitudeDegrees;
      nav_msg.altitude = GPS.altitude;
      nav_msg.position_covariance[0] = std::pow(GPS.HDOP,2);
      nav_msg.position_covariance[4] = std::pow(GPS.HDOP,2);
      nav_msg.position_covariance[8] = std::pow(2 * GPS.HDOP, 2);

      nav_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    } else {
      nav_msg.status.status = -1;
    }
    num_sats_msg.data = GPS.satellites;
    nav.publish(&nav_msg);
    num_sats.publish(&num_sats_msg);
  }

  nh.spinOnce();
}
