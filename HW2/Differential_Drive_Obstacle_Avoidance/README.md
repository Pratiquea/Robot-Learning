# Obstacle_Avoidance_ROS
Project in action : [Drive link](https://drive.google.com/file/d/1wfJBv8PTSks5bD7nhNxw_4pF6g-Ke84g/view)

## Cloning the project
   
   Create a new directory in ```<catkin_workspace>/src/Differential_drive``` and clone all of project files to that folder. 
   
   sample commands:
   
   1. ```mkdir ~/catkin_ws/src/Differential_drive```
   2. ```cd ~/catkin_ws/src/Differential_drive```
   3. ```git clone https://github.com/Pratiquea/Robot-Learning/tree/master/HW2/Differential_Drive_Obstacle_Avoidance```
   4. ```cd ~/catkin_ws```
   5. ```catkin_make```

## Step by Step ROS Command Explanation

1. First you need to Launch the ROS node.

   ```roslaunch testbot_description testbot_gazebo.launch```


2. Now you need to run the python script that is created to listen sensor data from our virtual robot and move the robot.
   
   ```rosrun testbot_description obs_avoid.py ```
   

