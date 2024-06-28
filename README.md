# differential_robot_sim

--------------------------------A Differential Robot Simulation in ROS and Gazebo--------------------------------------

About the project

A differential robot with two drivable wheels at the front and two casters at the rear is used. The robot has a 2D Lidar at the front, an RGBD camera and a screen attached to a pole.

The package contains the following,

(1) Robot description (urdf) and world
(2) Meshes and materials (for Hokuyo Lidar and Kinect camera)
(3) Config and Launch files (explained in detail in later sections)
(4) src: the scripts

The robot can be moved using teleop keyboard

Additional info:

(1) During autonomous movement, the robot can be enabled or disabled using a ROS service
(2) The robot can perform SLAM (for example using the gmapping package)
(3) The movement of the robot, map being built (if gmapping is installed) and the camera feed can be visualized in Rviz.

How to build

Clone the repository into your workspace's src folder and then run

(1) cd /your_ws

(2) catkin_make

Install teleop-twist-keyboard and slam-gmapping packages to use keyboard operation and LiDAR SLAM respectively. The following commands can be used to install the packages. Replace distro with your ros distro name

    sudo apt-get install ros-noetic-teleop-twist-keyboard
and

    sudo apt-get install ros-noetic-slam-gmapping

How to run (manual teleop)

(1) Spawn the robot

    roslaunch my_robot gazebo.launch
    
(2) Start gmapping

    roslaunch my_robot slam.launch
    
(3) Start teleop keyboard

    roslaunch my_robot teleoperator.launch
    
You can control the robot by using CLI for changing its topics and services, e.g., /cmd_vel
rostopic pub /cmd_vel geometry_msg/Twist ... REMEMBER: you can double click TAB for reminding you the CLI!!

Note: Source the devel space using "source devel/setup.bash", before launching the files

 
