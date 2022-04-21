# Project Alex

## Documentation

Documentations for individual modules are available within their respective directories

## General setup procedures

- On both devices:
  - `export ROS_IP=<LOCAL-IP-ADDRESS>`
  - `export ROS_MASTER_URI=<MASTER-IP-ADDRESS>`
- On RPi
  - Terminal window 1: `roslaunch rplidar_ros rplidar.launch` to start a rplidar node which will send scan results to the ros master node as defined by `ROS_MASTER_URI`
  - Terminal window 2: `cd <RPi module directory> && ./main` to start main RPi program
- On remote operator device(Ubuntu 18.06)
  - Terminal window 1: `roscore` to start a ros master node
  - Terminal window 2: `roslaunch rplidar_ros view_slam.launch` to start a slam ndode, which subscribes to the same topic as the rplidar node to receive scan results

### Modules

- AVC - Arduino Uno
- RPi - Rasberry Pi
- RPLidar
- TLS Client
- TLS Server
