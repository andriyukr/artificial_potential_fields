# artificial_potential_fields
ROS package for reactive obstacle avoidance using artificial potential fields.

Set `local_position/tf/send = true` in `/mavros/launch/px4_config.yaml` to get TF from `map` frame to `base_link` frame.

Terminal 1:

      roslaunch px4 mavros_posix_sitl.launch

Terminal 2:

      roslaunch artificial_potential_fields px4_simulation.launch

Gazebo:

![ ](https://github.com/andriyukr/artificial_potential_fields/blob/master/Results/gazebo.gif)

rviz:

![ ](https://github.com/andriyukr/artificial_potential_fields/blob/master/Results/rviz.png)
