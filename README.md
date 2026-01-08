# Reinforcement_Learning_UAV

ros2 run uav_rl_agent rl_agent_node.py \
  --ros-args \
  -p checkpoint_dir:=/path/to/ese651_project/checkpoints/<timestamp_folder> \
  -p vicon_pose_topic:=/vicon/<your_drone>/<your_drone>/pose \
  -p vicon_twist_topic:=/vicon/<your_drone>/<your_drone>/twist \
  -p track_mode:=square \
  -p square_center_xy:="[0.0, 0.0]" \
  -p square_side_m:=5.0 \
  -p track_z_m:=1.2 \
  -p target_square_size_m:=<MATCH_SIM_VALUE>

ros2 run uav_rl_agent arm_once.py --ros-args -p aux1_arm_us:=1100
ros2 run uav_rl_agent disarm_once.py --ros-args -p aux1_disarm_us:=900

ros2 run uav_rl_agent disarm_once.py --ros-args -p aux1_disarm_us:=900
ros2 run uav_rl_agent disarm_once.py --ros-args -p publish_reps:=10 -p publish_period_s:=0.03


