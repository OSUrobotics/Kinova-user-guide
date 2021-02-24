# Kinova-user-guide
This is a guide on how to control the kinova arm using ROS MoveIt Package


# Creating Workspace

To avoid running into build error issues, I recommend creating a dedicated workspace for all kinova packages. Run the following commands in your terminal to create a new catkin workspace:

* Source your environment


      $ source /opt/ros/melodic/setup.bash #replace melodic with your ros distro



* Create and build workspace


      $ mkdir -p ~/kinova_ws/src
      $ cd ~/kinova_ws/
      $ catkin_make


* Source the setup file in the workspace


      $ cd ~/kinova_ws/
      $ source devel/setup.bash


# Installing Packages

* Install [MoveIt package](https://moveit.ros.org/)  from the terminal using the following:

      $ sudo apt-get install moveit*


* Git clone the [kinova-ros package](https://github.com/Kinovarobotics/kinova-ros/blob/master/README.md) in the src folder of your workspace and catkin_make using the following commands:


      $ cd ~/kinova_ws/src
      $ git clone https://github.com/Kinovarobotics/kinova-ros.git kinova-ros
      $ cd ~/kinova_ws
      $ catkin_make


* To be able to run the arm through USB, run the following command:


      $ sudo cp kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/

* Create a new package called kinova_scripts inside the kinova-ros folder using the following commands:


      $ cd ~/kinova_ws/src/kinova-ros/
      $ catkin_create_pkg kinova_scripts std_msgs rospy roscpp
      $ cd ~/kinova_ws
      $ catkin_make

* Put the [kinova_path_planning.py](https://drive.google.com/open?id=1pvvcQMwMY1uACKpUQSM-u-eN--IOXOdO) file inside the src folder of kinova_scripts package. This script uses the MoveIt Python API to communicate with the arm. 


# Running the Kinova Arm in Simulation

To run the kinova arm virtually,  run the following command on a new terminal:


    $ roslaunch j2s7s300_moveit_config j2s7s300_virtual_robot_demo.launch


This will also launch rviz which will display the trajectory of the arm.


# Running the Real Kinova Arm

*Before running each of the following commands, don’t forget to source the setup.bash file in your workspace.*

* In a new terminal, run the following command to start the arm:


      $ roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300


* In a seperate new terminal, run the following terminal to start MoveIt:


      $ roslaunch j2s7s300_moveit_config j2s7s300_demo.launch 


* Navigate to the kinova_scripts directory and run the ''kinova_path_planning.py'' using:


      $ cd kinova_ws/src/kinova-ros/kinova_scripts/src/
      $ ./kinova_path_planning.py # Make sure the script is executable

      or $ python kinova_path_planning.py

 

# Using kinova_path_planning.py file
* The *MoveRobot()* class uses the moveit API to send waypoints to the arm. 
* *MoveRobot().go_to_goal([x, y, z, roll, pitch, yaw])* takes in the cartesian coordinates and orientation of the end effector in a list. *You can also give it the quaternion coordinates instead of roll, pitch and yaw. *
* *MoveRobot().go_to _joint_state([joint1, joint2,.....joint7])* takes in the 7 joint angles of the arm in a list form. 
* Call these functions in '''MoveRobot().move()

# Relevant rostopics

* Run the following command to look at the all the topics the kinova and moveit node is publishing at:


      $ rostopic list


* To print the current end effector pose, run the following in a new terminal window:


      $ rostopic echo /j2s7s300_driver/out/tool_pose


* To print the current joint angles, run:


      $ rostopic echo /j2s7s300_driver/out/joint_state


----
For questions, contact:

Nuha Nishat

Email: nishatn@oregonstate.edu


![](https://i.imgflip.com/3volxw.jpg)
