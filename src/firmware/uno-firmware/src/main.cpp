
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ctime>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdio.h>

#include <ros.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <cmath>


HardwareSerial mySerial = HardwareSerial();
Adafruit_GPS GPS(&mySerial);
Adafruit_BNO055 bno = Adafruit_BNO055(55);

ros::NodeHandle nh;

std_msgs::Int32 num_sats_msg;
sensor_msgs::NavSatFix nav_msg;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;

ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher nav("gps", &nav_msg);
ros::Publisher num_sats("num_sats", &num_sats_msg);
ros::Publisher mag("mag", &mag_msg);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(nav);
  nh.advertise(num_sats);
  nh.advertise(imu_pub);
  nh.advertise(mag);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  imu_msg.header.seq = 0;
  imu_msg.header.frame_id = "imu_frame";
  imu_msg.orientation_covariance[0] = -1;
  imu_msg.angular_velocity_covariance[0] = -1;
  imu_msg.linear_acceleration_covariance[0] = -1;

  mag_msg.header.seq = 0;
  mag_msg.header.frame_id = "imu_frame";
  memset(&mag_msg.magnetic_field_covariance, 0, sizeof(mag_msg.magnetic_field_covariance));


  bno.begin();
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  num_sats_msg.data = 0;
  nav_msg.header.frame_id = "gps_frame";
  nav_msg.header.seq = 0;
  nav_msg.status.status = -1;
  nav_msg.status.service = 1;

}

void serialEvent1() {
  digitalWrite(13, 1);
  while (mySerial.available()) {
    GPS.read();
  }
  digitalWrite(13, 0);
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

  imu::Quaternion quat = bno.getQuat();
  quat.normalize();

  imu_msg.orientation.w = quat.w();
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();

  auto gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_msg.angular_velocity.x = gyro[0];
  imu_msg.angular_velocity.y = gyro[1];
  imu_msg.angular_velocity.z = gyro[2];

  auto accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu_msg.linear_acceleration.x = accel[0];
  imu_msg.linear_acceleration.y = accel[1];
  imu_msg.linear_acceleration.z = accel[2];

  imu_msg.header.stamp = nh.now();
  imu_msg.header.seq++;
  imu_pub.publish(&imu_msg);

  auto mag_field = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  mag_msg.magnetic_field.x = mag_field[0] / 1000000;
  mag_msg.magnetic_field.y = mag_field[1] / 1000000;
  mag_msg.magnetic_field.z = mag_field[2] / 1000000;
  mag_msg.header.stamp = nh.now();
  mag_msg.header.seq++;
  mag.publish(&mag_msg);

  nh.spinOnce();
}
