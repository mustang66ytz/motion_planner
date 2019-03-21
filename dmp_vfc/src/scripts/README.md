
### Ginger interface (Python) Setup 

* [Workspace] - make a new directory for your workspace.
```sh
$ cd /PATH/TO/Directory
$ mkdir WORKSPACE/src
```
* [Copy package] - copy three packages: visualcontrol, xr1controllerol, xr1controllerros to source(/src) directory.
* [CMake] - go to workspace root directory, then:
```sh
$ catkin_make
$ source devel/setup.bash
```
* [V_Rep] - go to xrcontrollerros directory, then:
```sh
$ ./launch_vrep.sh
```
* [IK_Simulator setup] 
```sh
$ rosrun xrcontrollerol xr1controllerIK_simulator 
```
* [Python interface setup] 
```sh
$ rosrun visualcontrol manipulation 
```


-----------------------------------------------------------------------

### Mocap Setup 

MAKE SURE ALL THE IMUS ARE FULLY CHARGED BEFORE CONNECTING.

* [Vrep_test] - Download vrep_ros control interface here: URL: https://github.com/Rayckey/XR1_vrep_ros.
* [Vrep_test] - Setup vrep_test.
* [Ethernet config] - Disable the Wi-Fi network and configure the Networking by:
```sh
$ ifconfig  # check the network configuration. Find one with enpXXX.
$ sudo ifconfig enpXXX 192.168.1.111  # allocate ip address
```
* [Router] - Plug in the charger and connect the router using Ethernet cable.
* [Source] - cd to the vrep_test workspace. And then:
```sh
$ source devel/setup.bash 
$ cd ~/src/vrep_test
$ ./XR1_vrep.sh
```
* [IMU Setup] - Make sure IMU components are fully charged. Press OPEN button on each IMU side. (Blue light blink)
* [IMU Initialization] Rotate the IMU (roll, pitch, yaw), until blue light change to green light. 
* [My plugin] - Wear all the IMUs properly and click 'InitIMU' in the interface. 
* [Start playing]

