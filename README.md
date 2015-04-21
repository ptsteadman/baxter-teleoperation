## Baxter Teleoperation using Kinect and the Oculus Rift

This repository contains code for an interactive console that allows 
you to send commands to Baxter, including starting teleoperation.

##### Setup:

First, enter the Baxter environment.
If you want to use the simulator: 

        cd ~/ros_ws
        . baxter.sh sim
        roslaunch baxter_gazebo baxter_world.launch

If you want to use the real robot:

        cd ~/ros_ws
        . baxter.sh

Second, start the kinect client in a new terminal tab:

        roslaunch openni_launch openni.launch

Third, start tracking in another termial tab:

        roslaunch openni_tracker openni_tracker.launch

Finally, enable the robot and start the console.  

        rosrun baxter_tools enable_robot.py -e
        python baxter_console.py

You can then run commands at the `>>>` prompt.  

### Commands:
- list 
- of
- commands

