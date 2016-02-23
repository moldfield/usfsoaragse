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
Now start `rqt` from laptop (or secondary device with ros desktop tools) and select `plugins` then `visualization` then `image view`.
```
$	rqt
```

## Starting up the xbee
