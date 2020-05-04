# planning_competition

## Authors

 1. Sri Manika Makam
 2. Akanksha Patel
 3. Aruna Baijal
 
## Setup 
In your .bashrc file, include the following statements and source it.

```
source /opt/ros/kinetic/setup.bash
source ~/car_ws/devel/setup.bash
```

Clone the repository and run the following commands.

```
$ cd ~/car_ws/
$ catkin_make
```
 
## Instructions to run

Run the following three commands in order in three different terminals.

```
roslaunch toycar display.launch
roslaunch robot_control robot_control.launch 
roslaunch toycar gazebo.launch
```

## Output

Default inputs: 
start point = ()
goal point = ()

The time taken by the algorithm to find the shortest path for default inputs is approximately 9 seconds.
The time taken for simulation in gazebo is approximately 6 minutes

Text file named 'out.txt' has all the velocity and angles for every 2 seconds in the found path.

The video outputs for mapping and gazebo simulations can be accessed here:
https://drive.google.com/drive/folders/1AgRXgyogmuREwNg90ecDwI6LoKeRHZlb?usp=sharing

