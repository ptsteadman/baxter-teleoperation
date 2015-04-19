Need to run ./baxter.sh in each new shell
extract contents into ~/ros_ws/src/hri-dp2

New terminal

cd ~/ros_ws
./baxter.sh
roslaunch openni_launch openni.launch

New terminal

cd ~/ros_ws
./baxter.sh
roslaunch openni_tracker openni_tracker.launch

New terminal

cd ~/ros_ws
./baxter.sh
cd ~/ros_ws/src/hri-dp2
rosrun baxter_tools enable_robot.py -e
python teleoperate.py --user 1 --rate 10 --mirrored:=true

To get presentation remotes to work, plug them into another machine and run the key remap script. The keyboard remap remaps the .(period) key, making typing commands difficult, so execute the gripper control script first, escape, remap keys, then use the up arrow to execute the gripper script again.
Place the remap_keys.sh script in the ~/ros_ws folder on the new machine. This machine will have to be rebooted to undo the key remapping.

New terminal on new machine

cd ~/ros_ws/
./baxter.sh
rosrun baxter_examples gripper_keyboard.py
ctrl-c
bash remap_keys.sh
rosrun baxter_examples gripper_keyboard.py (via up arrow in terminal)

The top row corresponds to the right gripper and the bottom to the left (on both remotes).
			
			laser_pointer 			(do not shine in observers' eyes)

		right.close		right.open

		left.open		left.close

		

