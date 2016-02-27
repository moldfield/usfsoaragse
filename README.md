# USFSOAR AGSE

|  Section      | Currently Used|
| ------------- |:-------------:|
|  Hardware     | Raspberry Pi  |
|       OS      | Ubuntu 14.0.4       |
|               |      ROS      |
| zebra stripes | are neat      |

##Usage

Before using any ros commands in any shell first run
```
source /opt/ros/indigo/setup.bash
``` 
# To check environment variables
```
$	printenv | grep ROS
```
## Running on multiple computers
To connect multiple computers with ros over ssh, make sure all computers know where the master node is and that the master node can find the others by ip address.

On the roscore master node in the `~/.bashrc` file so that these commands load on every shell. The basic premise here is to let every node know who is the master ip node running roscore then in the second line broadcast the computers own ip.
```
IP1: 	192.168.1.42 	# raspberry pi
IP2:	192.168.1.173 	# laptop
```
In pi `~/.bashrc`
```
export ROS_MASTER_URI=http://192.168.1.42:11311
export ROS_IP=192.168.1.42
```
In the laptop `~/.bashrc`
```
export ROS_MASTER_URI=http://192.168.1.42:11311
export ROS_IP=192.168.1.173
```
This will only need to be done once on every machine as long as the ip addresses remain the same.


## Starting up the kinect

Remove the gspca modules so that the user-mode libfreenect drive can take over.
```
$  	sudo modprobe -r gspca_kinect

$  	sudo modprobe -r gspca_main
```
Launch files to open an OpenNI device and load all nodelets to convert raw depth/RGB/IR streams to depth images, disparity images, and (registered) point clouds.
```
$  	roslaunch openni_launch openni.launch
```
Now start `rqt` from laptop, or secondary device with ros desktop tools, and select `plugins` then `visualization` then `image view`.
```
$	rqt
```
## Finding ROS Drivers

Drivers for most things that will be needed can be found at https://github.com/ros-drivers/ . 

Steps to install:
	1. Find driver (eg: https://github.com/ros-drivers/rosserial.git).
	2. Clone into the `src` folder in your catkin workspace directory.
	3. `cd` back to workspace directory.
	4. Run `$ catkin_make`
	5. Run `$ catkin_make install`


## Creating and Sourcing a Workspace. 

First create `ros_workspace` and `ros_workspace/src`.
```
$	cd ros_workspace/src
$	git clone https://github.com/ros-drivers/rosserial.git
$	cd ros_workspace
$	catkin_make
$	catkin_make install
$	source ros_workspace/install/setup.bash
```


## XBee Setup

### Installation

Install `rosserial` and `rosserial_xbee` for command line capabilities, this only needs to be done once on any given machine.
```
$	sudo apt-get -y install ros-indigo-rosserial
$	sudo apt-get install ros-indigo-rosserial-xbee
```
### Find the XBee

List all USB devices
```
$	lsusb | grep Future
```
Here are some ways that I have seen them listed 
	`Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC`
	`Future Technology Devices International, Ltd Bridge(I2C/SPI/UART/FIFO)`
### Setup Coordinator and nodes

Use `-C` for Coordinater setup. This is the master node and should always have an id of `0`
```
$	rosrun rosserial_xbee setup_xbee.py -C /dev/ttyUSB0 0
```
Setup any other node `N` where `N = 1,2,3,...`
```
$	rosrun rosserial_xbee setup_xbee.py /dev/ttyUSB0 N
$	rosrun rosserial_xbee setup_xbee.py /dev/ttyUSB0 1 # Node 1 for example
```

All rosserial_xbee network coordinators should have an ID of 0. It also sets up some default configurations.

	-API mode - `2` (a binary protocol with escape characters)
	-Baud rate - `57600 baud`
	-Network ID - `1331`
	-Frequency Channel - `D`
## Servo driver

16-Channel 12-bit PWM/Servo Driver http://adafru.it/815
