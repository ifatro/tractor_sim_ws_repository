Simulation description
----------------------------------------

The simulation implements a four wheel skid driving control robot that receives a geographic
point command ([latitude longitude]-wgs84) from the GUI prompt, then it plans a path to the
target that includes the distance[meter] and the bearing angle[deg] from the current pose of
the robot to the requested geographic point of the target and drives towards it with a
distance error<1[meter].
The robot is able to receive a new setpoint over the course of driving to a current setpoint
command, in this case the robot stops the path to the current command after decelerating,
calculates the path to the updated target and sends a command to the wheels.
The 4 wheeled robot has a controlled driving system that converts distance[meter] and
bearing[deg] commands to torque commands [rad/sec] with an acceleration and deceleration
simplified logic.
The robot receives its full odometry from a simulated IMU that serves as the Robot’s main
sensor and feedback to the control.
The SERVO control of the robot from the torque command to the actual movement of the
robot wheels is implemented as a GAZEBO plugin (hinge joints velocity controllers).
The navigation of the robot includes an IMU that is also implemented from a GAZEBO
plugin.

   

Source code 
----------------------------------------
The source code can be found in the gitHub repository tractor_sim_ws_repository in the following link:
https://github.com/ifatro/tractor_sim_ws_repository.git

The source code includes:

gazebo_tractor_sim_pkg ; 
ros_tractor_sim_pkg ; 
CMakeLists.txt ; 
README.md ; 

In order to run the simulation all 1-4 need to be downloaded under a Workspace/src folder and source the workspace to the bashrc.


Implementation
----------------------------------------
The simulation is implemented using the following 5 ROS nodes, each node has a different functionality and connections to other nodes:


GUI
----------------------------------------
main.python - The node receives the robot and target locations (wgs84) and draws them on a geographic map.

robot_path_planning_node
----------------------------------------
The node calculates the robot bearing angle and distance to the required target;    
Rate: 10[Hz]
file: robot_path_planning.cpp


robot_navigation_node
----------------------------------------
The node receives the IMU measurements and extracts the robot full odometry (angular & linear);
Rate:100[Hz]
file:robot_navigation.cpp

robot_control_node
--------------------------------------
The node receives the required robot’s path from the path planner node and the robot’s odometry from the robot’s navigation node and calculates the commands to the controller wheels;
Rate:30[Hz]
file:robot_control.cpp


sim_ground_truth_node*
----------------------
The node receives the robot ground truth odometry and calculates the robot's true location to be drawn in the GUI app;
*The sim_ground_truth_node is not connected to the robot’s nodes (navigation, control, path planning) and serves only to present the robot’s true location on the GUI geographic map;
Rate:10[Hz]
file:robot_sim_ground_truth.cpp



Publishers and subscribers of each node:
----------------------------------------
/target_location;
/robot_location;
/robot_path_planning_node;
/robot_target_path;
/robot_estimated_location;
/in_turn;
/robot_navigation_node;
/robot_estimated_location;
/robot_odometry;
/robot_bearing_imu;
/imu;
/robot_control_node;
/left_wheel_2_hinge_joint_velocity_controller/command;
/left_wheel_hinge_joint_velocity_controller/command;
/right_wheel_2_hinge_joint_velocity_controller/command;
/right_wheel_hinge_joint_velocity_controller/command;
/curr_dist;
/cmd_dist;
/time_from_start_driving_ahead;
/in_turn;
/sim_ground_truth_node;
/ground_truth/state;

Simulation compilation
----------------------------------------
The simulation is implemented in a workspace called tractor_sim_ws.
This package includes the gazebo package to launch the GAZEBO simulation, and the ROS package to launch the GUI and the ROS code. both packages are located  under the src/ folder:

The packages are called:

gazebo_tractor_sim_pkg - launchs the GAZEBO simulation
ros_tractor_sim_pkg - launches the GUI and ROS system.

The compilation command for both packages is:
 


For the compilation there may be additional packages needed to be installed.

Installation of required packages commands:

  sudo apt install ros-noetic-joint-state-controller
  sudo apt install ros-noetic-velocity-controllers
  sudo apt-get install gazebo11-plugin-base
  sudo apt install ros-noetic-hector-gazebo-plugins
  sudo apt-get install ros-noetic-navigation



Simulation running
----------------------------------------

The simulation was developed for Ubuntu 20.04  with ROS noetic.

In order to run the simulation it is needed to launch the couple of  the launch files:
world.launch - launches the tractor model and the world in the gazebo environment.
tractor_sim.launch - launches the ROS noes, parameters and GUI.
Then both the GAZEBO and the GUI are opened and the simulation can be run.





