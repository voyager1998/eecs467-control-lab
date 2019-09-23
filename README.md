# eecs467-control-lab
Univ of Michigan EECS 467: Autonomous Robotics - Control Lab 

## M-Bot Structure

M-Bot contains a Lidar connected to a Rasperry Pi serving as a high-level controller, and a BeagleBone board serving as a low-level motor driver.

## Code Structure

* In `botlab-f19`, two versions of `make` commands are available: `make mbot-only` and `make laptop-only`. The former compiles files related to the Rasperry Pi and Lidar but nothing related to the Vx graphical visualization, while the latter only compiles everything related to the graphical visualization.

* In `mobilebot-f19`, codes are only prepared for low-level BeagleBone board and motor drivers. [Robot Control Library](http://strawsondesign.com/docs/librobotcontrol/) is installed by default on the BeagleBone board but should be **manually installed onto the PC** if you wish to compile this folder on laptop.
