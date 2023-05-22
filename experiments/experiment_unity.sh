for i in `seq 5`
do
  roslaunch v_tsukuba nav_cloning_sim.launch script:=nav_cloning_with_direction_node.py mode:=selected_training world_name:=tsudanuma_photo.world map_file:=tsukuba_unity waypoints_file:=tsukuba_unity_way.yaml dist_err:=0.8 initial_pose_x:=-0.00123 initial_pose_y:=0.301 initial_pose_a:=0.0 use_waypoint_nav:=true robot_x:=-16 robot_y:=-42 robot_Y:=-1.57
  sleep 10
done