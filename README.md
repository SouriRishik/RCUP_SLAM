# RCUP_SLAM
## Scripts folder
- send.py: sends pwm to wheels after establishing communication with Ardunio
- recieve.py: receives encoder data via another Ardunio
- cmd_vel_to_odom.py: subcribes to cmd_vel and gets the data and convert to Odom and publish in /odom
- odom_to_base_transform.py: 
## Launch folder
- view_sllidar_a2m8_launch.py - this is a launch file for rotating the LIDAR and getting the data. It has dynamic transform for odom to base_footprint and static transform of base_footprint to base_link and base_link to laser. It has got Slam_toolbox and Nav2 bringup launch description.
