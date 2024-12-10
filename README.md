# RCUP_SLAM
## Scripts folder
  send.py - sends pwm to wheels after establishing communication with Ardunio
  recieve.py - receives encoder data via another Ardunio
  cmd_vel_to_odom.py - subcribes to cmd_vel and gets the data and convert to Odom and publish in /odom
  odom_to_base_transform.py
