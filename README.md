# arm_interface
Package containing the files and nodes to interact with the Dynamixel-based robotic arm developed in PRISMA Lab for use on the ANYmal robot. All the code developed is for use with ROS Noetic.

## Dependencies
In your workspace, download and install the DynamixelSDK repository (be sure to checkout to the noetic-devel branch)

## Deployment as a mobile manipulator
To deploy the arm on the mobile manipulator, a specific hardware setup is needed.

First of all, the battery (1500 mAh, 5s battery with XT-60 connector) must be connected to the regulator input port, while the output port must be connected to the arm regulator supply port, being careful its breaker is on the OFF position (the ON position has a white dot on it). The input and output ports are different, to avoid wrong connections. NEVER CONNECT THE BATTERY TO THE ARM DIRECTLY AS THE VOLTAGE IS TOO HIGH.

The battery and regulator, along with the arm, must be affixed on the ANYmal robot. For the arm, use the provided screws to mount it on the payload interface. For the battery and regulator, use masking tape (Nastro carta) to cover the surfaces, and 3M to attach everything. The tested position for the battery and regulator is behind the emergency stop, next to the LiDAR, with the regulator on the battery for short operations. A more robust structure (e.g. custom PLA frame) or other suggestions are well appreciated.

To connect to the robot, be sure that the robot is off. Then, you have to connect an USB hub (or a USB-A to USB-C male adapter) to the robot's USB-C port, connected to the NPC, as well as the cable to the Dynamixel regulator. Once the connection is made, turn on the robot. If the connection is established, the regulator will turn on with a red LED (the board has its own red LED for the power supply, which is separate and is in no way an indicator of the connection). The board may stay unpowered for this connection. Check in the NPC if the robot sees the ttyUSB0 connection, after which you can test if the connection is actually working by sourcing the workspace, running 'rosrun arm_interface bulk_read' and check if the /joint_states_in topic reads anything. To check if the robot can enable the arm motors' torque, test with the 'rosrun arm_interface robot_interface' node if the arm gets stiff.

## Repo Structure
This repository currently provides the code for three different tasks: simulate the arm with a GUI joint control, launch the robot interface with the same GUI to control real joint values, and finally launch only the robot interface without the GUI for direct API control. These tasks contribute to mainly two workflows

### GUI joint control
The two GUI launch files (arm_sim_gui.launch and arm_interface_gui.launch) are meant to be used mainly for testing and debugging, allowing for quick control of the joint values. No other files are required for this. (arm_sim.launch is the same simulator without the GUI slider node — publish /joint_states_out yourself to drive it.)

### Replay mode on hardware
The third launch file (arm_interface.launch) is meant for API usage, which as of right now consists mainly in the "Replay mode" pipeline. For this, a series of steps is required.

#### Step 1: Record the movement
For this, the user needs to first run 'bulk_read', a node that simply reads the pose of the motor joints, converts them in radians and publishes them on the joint_states_in topic. Then, a rosbag of that state must be recorded of that topic while the arm is moved as desired. Keep in mind, the PID of the motors do not account for gravity, so compensate a bit by hand. Also, try to remember the initial position of the motion and keep it close to the final position for consistency.

#### Step 2: Launch the interface
After this, put the robot in the motion's initial position, and run the 'robot_interface' node. Check that the torque has been enabled in all motors and that the '/joint_states_in' is being published. Keep in mind, exiting this node should deactivate the torque to the motors. A stronger interruption is pressing the breaker on the board, but it's not as immediate.

#### Step 3: Replay the motion
In a different terminal, run the 'replay_bag' node, which prompts the user to input a folder. Then, the terminal will list all the bag files in said folder, prompting the user to choose between them via their list number (SHOULD be chronological but I remember also them not in chronological order). Finally, the user will be prompted to choose if a topic requires remapping. Agree with it and it will ask which topic to remap (/joint_states_in) and to which topic to remap (/joint_states_out).