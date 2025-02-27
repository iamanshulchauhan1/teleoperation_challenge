  #!/bin/bash
  rosdep install --from-paths src --ignore-src -r -y
  pip install --upgrade transforms3d
  source /opt/ros/humble/setup.bash
  source /root/ws_moveit/install/setup.bash
  
  colcon build --symlink-install
  source install/setup.bash

  # Launch Move Group
  ros2 launch dual_arm_servo_moveit_config move_group.launch.py &
  
  # Launch Left Arm Servo
  ros2 launch dual_arm_servo_moveit_config left_servo.launch.py &
  
  # Launch Right Arm Servo
  ros2 launch dual_arm_servo_moveit_config right_servo.launch.py &
  
  # Switch command type for left arm
  ros2 service call /left/arm_servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 2}" &
  
  # Switch command type for right arm
  ros2 service call /right/arm_servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 2}" &
  
  # Launch Teleoperation Node
  ros2 run teleop_dual_arm teleop_node  