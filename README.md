# youbot_velocity_interface
This is a prototype for a simple ROS based Publisher interface to control the YouBot base platform

How to run it :

Start ROS master

````console
$ roscore&
$ rosrun youbot_velocity_interface talker.py <filename_containing_velocities>.csv
````

The file should contain the following columns:

vx,vy,omega
