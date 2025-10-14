# 1. This repository 

# micro ros ESP32, mpu6050 modules

This tutorial treat serial communication.
|Demo Video|
|---|
||
# 2. Arrangement 
esp32, 6050 imu, 5pin-is also fine

# 3. Wiring 
ESP32 3V3 → MPU6050 VCC 
ESP32 GND → MPU6050 GND 
ESP32 D21 → MPU6050 SDA 
ESP32 D22 → MPU6050 SCL 
|Simulation|Real world|
|||
|||

# How to use 


### Refer to micro ros tutorial(https://micro.ros.org/docs/tutorials/core/first_application_linux/),
install micro-ros package
```
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
cd ~/
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```
|If you complete upper settings properly, you can see autocomplete using tap key|
|---|
||



### Refer to Esp32 idf(IoT Development Framework) tutorial(https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html),
Install Esp32 idf library, make sure to deactivate python virtual environment. ex) venv, conda
You don't have to follow all the tutorial. Just complete until step4, get_idf. )
```
# In my case, ubuntu 22.04
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

mkdir -p ~/esp
cd ~/esp
git clone -b v5.5.1 --recursive https://github.com/espressif/esp-idf.git
cd ~/esp/esp-idf
./install.sh all

# regist get_idf command in bash setting script.
echo "alias get_idf='. \$HOME/esp/esp-idf/export.sh'" >> ~/.bashrc
```
After installing esp32-idf, you can type "get_idf" command. get_idf will bring up idf python environments.

|If you complete upper settings properly you can see|
|---|
||


### Download freeRTOS esp32 idf setting 
This command will install firmware. After this command, you can see firmware folder .
```
cd ~/microros_ws
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
```
Add imu control module
```
cd ~/microros_ws/firmware/apps/
git clone "THIS REPOSITORY"
```

### Build and flash this module
```
ros2 run micro_ros_setup configure_firmware.sh my_app --transport serial
ros2 run micro_ros_setup build_firmware.sh
ros2 run micro_ros_setup flash_firmware.sh
```
Ihis process, put your heart and soul into it. Especially flashing the firmware might be wire issue. 
I used usb-c2c wire and usb-c to 5pin converter. If you use obsolete usb to micro usb cables, data fransformation will not able. Those wires are only for charging. If you have more problem fleshing memory, yous hould see trouble shooting chapter. When you flashing data, push reset button aggressively. 

|If you complete upper settings properly you can see|
|---|
||

### Build and test ros code
Build an agent.
```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
Get usb port.
```
ls /dev/serial/by-id/*
# In my case, /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```
Publish IMU data.
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```
If you push reset button you can see published imu data.

### Trouble shooting
In flashing error.
First,
```
lsusb
(connect or disconnect usb cable)
lsusb
```
check this command, if something apperar in bus, you selected correct cable.

Second, if your esp32 is wroom model, you might erase brltty.
refer to (https://www.reddit.com/r/pop_os/comments/uf54bi/how_to_remove_or_disable_brltty/)
brltty might disterb device driver 

Third, make sure soldering, hardware connection. Try reversing c wire. 

Fourth, if you overwrite the flasing memory, build file and esp32 should be clean.
```
rm -rf firmware/freertos_apps/microros_esp32_extensions/build
rm -rf firmware/freertos_apps/microros_esp32_extensions/install
esptool.py erase_flash
```

After flashing 
If you have problem reading imu data, 

First,
```
screen /dev/ttyUSB0 115200
```
you can add log and see what happen.

Second, get imu data without micro-ros.
```
cd ~/esp32_i2c_test
~/microros_ws/firmware/toolchain/esp-idf/export.sh
idf.py build flash monitor
```
you can use do this using esp_i2d_test folder, check soldering status. I found my soldering issue using this.

# Reference
https://micro.ros.org/
https://link.medium.com/JFof42RUwib.
https://medium.com/@SameerT009/connect-esp32-to-ros2-foxy-5f06e0cc64df
https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/
https://brltty.app/
https://github.com/micro-ROS/micro_ros_setup/issues/580
https://github.com/espressif/esptool/issues/626
