
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
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <cmath>


HardwareSerial mySerial = HardwareSerial();
Adafruit_GPS GPS(&mySerial);
Adafruit_BNO055 bno = Adafruit_BNO055(55);

ros::NodeHandle nh;

std_msgs::Int32 num_sats_msg;
std_msgs::UInt8MultiArray imu_status_msg;
sensor_msgs::NavSatFix nav_msg;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;

ros::Publisher imu_pub("imu/data", &imu_msg);
ros::Publisher nav("gps/raw", &nav_msg);
ros::Publisher num_sats("num_sats", &num_sats_msg);
ros::Publisher mag("imu/mag", &mag_msg);
ros::Publisher imu_status("imu/status", &imu_status_msg);

const char dim0_label[] = "status";

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
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 5 Hz update rate

  imu_msg.header.seq = 0;
  imu_msg.header.frame_id = "base_link";
  imu_msg.orientation_covariance[0] = .001;
  imu_msg.orientation_covariance[4] = .001;
  imu_msg.orientation_covariance[8] = .001;

  imu_msg.angular_velocity_covariance[0] = 1;
  imu_msg.angular_velocity_covariance[4] = 1;
  imu_msg.angular_velocity_covariance[8] = 1;

  imu_msg.linear_acceleration_covariance[0] = 1;
  imu_msg.linear_acceleration_covariance[4] = 1;
  imu_msg.linear_acceleration_covariance[8] = 1;

  mag_msg.header.seq = 0;
  mag_msg.header.frame_id = "base_link";
  memset(&mag_msg.magnetic_field_covariance, 0, sizeof(mag_msg.magnetic_field_covariance));

  bno.begin();
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  num_sats_msg.data = 0;
  nav_msg.header.frame_id = "map";
  nav_msg.header.seq = 0;
  nav_msg.status.status = -1;
  nav_msg.status.service = 1;

  imu_status_msg.data =  new uint8_t[4];
  imu_status_msg.data_length = 4;
  imu_status_msg.layout.data_offset = 0;
  imu_status_msg.layout.dim_length = 1;
  imu_status_msg.layout.dim = new std_msgs::MultiArrayDimension;
  imu_status_msg.layout.dim[0].size = 4;
  imu_status_msg.layout.dim[0].stride = 1;
  imu_status_msg.layout.dim[0].label = dim0_label;
  nh.advertise(imu_status);

}

void serialEvent1() {

  digitalWrite(13, 0);
  while (mySerial.available()) {
    GPS.read();
  }
  digitalWrite(13, 1);
}

void loop()                     // run over and over again
{

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      goto IMU;  // we can fail to parse a sentence in which case we should just wait for another

    struct tm timeinfo;
    timeinfo.tm_year = GPS.year + 100;
    timeinfo.tm_mon = GPS.month - 1;
    timeinfo.tm_mday = GPS.day;
    timeinfo.tm_hour = GPS.hour;
    timeinfo.tm_min = GPS.minute;
    timeinfo.tm_sec = GPS.seconds;
    time_t sec = mktime(&timeinfo);

    // if we already published a message with the same timestamp, break out.
    if ((sec == nav_msg.header.stamp.sec) && ((GPS.milliseconds * 1000000) == nav_msg.header.stamp.nsec))
       goto IMU;
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
      // nav_msg.position_covariance_type = 0;
      nav_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    } else {
      nav_msg.status.status = -1;
    }
    num_sats_msg.data = GPS.satellites;

    nav.publish(&nav_msg);
    num_sats.publish(&num_sats_msg);
  }
  IMU:
  imu::Quaternion quat = bno.getQuat();
  imu_msg.orientation.w = quat.w();
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();

  auto gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_msg.angular_velocity.x = gyro[0];
  imu_msg.angular_velocity.y = gyro[1];
  imu_msg.angular_velocity.z = gyro[2];

  auto accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu_msg.header.stamp = nh.now();
  imu_msg.linear_acceleration.x = accel[0];
  imu_msg.linear_acceleration.y = accel[1];
  imu_msg.linear_acceleration.z = accel[2];

  auto mag_field = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  mag_msg.header.stamp = nh.now();
  mag_msg.magnetic_field.x = mag_field[0] / 1000000;
  mag_msg.magnetic_field.y = mag_field[1] / 1000000;
  mag_msg.magnetic_field.z = mag_field[2] / 1000000;

  imu_msg.header.seq++;
  imu_pub.publish(&imu_msg);
  mag_msg.header.seq++;
  mag.publish(&mag_msg);


  bno.getCalibration(imu_status_msg.data, imu_status_msg.data + 1, imu_status_msg.data + 2, imu_status_msg.data + 3);

  imu_status.publish(&imu_status_msg);
  static uint8_t light;
  digitalWrite(13, light=!light);

  nh.spinOnce();
}
