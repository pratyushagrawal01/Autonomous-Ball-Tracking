Welcome to the autonomous ball tracking algorithm using opencv in ros2

Setup --> 

Download the mini_project folder in ros2 and store it in the ros2_ws folder. 
Download the unit_sphere folder and store it in models folder in .gazebo 
The present file is set to follow a red ball. 

To run the present setup: 
1) Open a terminal and go to the src folder in ros2_ws and colcon build
2) launch the gazebo file using -->  ros2 launch mini_project gazebo.launch.py
3) place the red ball in the sandbox.
4) open another terminal and run the command --> ros2 run mini_project ball
