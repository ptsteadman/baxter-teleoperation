## HRI DP2

This repository contains code for Human Robot Interaction Design Project Two, 
by Patrick Steadman, Tauhidur Rahman, and  Moumita Basuroychowdhury.

##### To start teleoperation using the simulator:

First, start the simulated baxter environment and enable the robot:

        cd ~/ros_ws
        . baxter.sh sim
        roslaunch baxter_gazebo baxter_world.launch


Second, start the kinect client in a new terminal tab:

        roslaunch openni_launch openni.launch

Third, start tracking in another termial tab:

        roslaunch openni_tracker openni_tracker.launch

Finally, enable the robot andstart teleoperation.  
The skeleton position tracking rate and the kinect 
user number can be specified:

        rosrun baxter_tools enable_robot.py -e
        python teleoperate.py --user 1 --rate 10 --mirrored false

