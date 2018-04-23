# ft_sensor_monitor
Code for reading from the force torque sensor

## Setup
```bash
sudo chmod a+rw /dev/ttyUSB0
```
This to to set up correct permissions for device. There is probably a way to do this using udev but unsure. 

## Running
Make sure you plug in the force torque sensor into an available USB port and it is switched on. 
```bash
$ cd <catkin_ws>
$ source devel/setup.bash
# After a ROS master has been started
$ rosrun ft_sensor_monitor force_torque_server
```