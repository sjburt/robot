DUMB ROBOT STUFF
================

whats here:
----------

`src/firmware/uno-firmware`: firmware for a teensy or arduino, uses platformio
                           to build

`src/scratch`: misc launch files

`src/rosserial`: copy of rosserial from  https://github.com/ros-drivers/rosserial.git

hardware
--------

  teensy 3.2 or one of the larger arduinos, but teensy is better.
  other boards might work.

  Adafruit or similar bno555 breakout

  adafruit or similar mk2777 gps breakout


how do
------
  install ros:
  ```
    sudo apt install ros-melodic-desktop-full ros-melodic-robot-localization
  ```
  initialize the workspace
  ```
    catkin_make
  ```
  run these or add them to your `.bashrc`
  ```
    source /opt/ros/melodic/setup.bash
    source devel/setup.bash
  ```

  build the messages (only need to do this once unless you add more messages):
  ```
     rosrun rosserial_arduino make_libraries.py devel/rosserial_arduino/msg          
  ```

  Then use Platformio to build the firmware and load it. Change .platformio if
  you aren't using a teensy.

  then
  ```
    roslaunch src/scratch/robot-loc.launch
  ```

hardware do
-----------
  wire the bno555 to the i2c port (teensy: pin 18 SDA0 pin 19 SCL0)

  wire the gps breakout to the teensy hardware uart1 (RX pin 0 to TX on breakout).

  If you have an uno you could try software serial, it might be OK.
