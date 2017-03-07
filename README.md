
 To shwitch to joint impedance control

rosservice call /right_arm/controller_manager/switch_controller "start_controllers: ['joint_impedance_controller']
stop_controllers: ['joint_trajectory_controller', 'stiffness_trajectory_controller', 'damping_trajectory_controller', 'add_torque_trajectory_controller']
strictness: 2"

rosservice call /left_arm/controller_manager/switch_controller "start_controllers: ['joint_impedance_controller']
stop_controllers: ['joint_trajectory_controller', 'stiffness_trajectory_controller', 'damping_trajectory_controller', 'add_torque_trajectory_controller']
strictness: 2"

to switch to trajectory controller. This is useful to go home

rosservice call /right_arm/controller_manager/switch_controller "stop_controllers: ['joint_impedance_controller']
start_controllers: ['joint_trajectory_controller', 'stiffness_trajectory_controller', 'damping_trajectory_controller', 'add_torque_trajectory_controller']
strictness: 2"

rosservice call /left_arm/controller_manager/switch_controller "stop_controllers: ['joint_impedance_controller']
start_controllers: ['joint_trajectory_controller', 'stiffness_trajectory_controller', 'damping_trajectory_controller', 'add_torque_trajectory_controller']
strictness: 2"




rostopic pub /right_arm/joint_trajectory_controller/command trajectory_msgs/JointTrajectory "header:  
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['right_arm_a1_joint', 'right_arm_a2_joint', 'right_arm_a3_joint', 'right_arm_a4_joint', 'right_arm_a5_joint', 'right_arm_a6_joint', 'right_arm_e1_joint']
points:
- positions: [-0.77, 0.63, -1.42, 0.0, 0.92, 0.0] 
  velocities: [] 
  accelerations: [] 
  effort: [] 
  time_from_start: {secs: 1.0, nsecs: 0}"



rostopic pub /left_arm/joint_trajectory_controller/command trajectory_msgs/JointTrajectory "header:  
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['left_arm_a1_joint', 'left_arm_a2_joint', 'left_arm_a3_joint', 'left_arm_a4_joint', 'left_arm_a5_joint', 'left_arm_a6_joint', 'left_arm_e1_joint']
points:
- positions: [0.77, 0.46, -0, 1.43, 0.0, 0.0] 
  velocities: [] 
  accelerations: [] 
  effort: [] 
  time_from_start: {secs: 1.0, nsecs: 0}"
