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
  - Terminal window 2: `roslaunch rplidar_ros view_slam.launch` to start a slam ndode

## Main aim

To build robotic vehicle named Alex which
can be deployed to locate and rescue survivors following a natural/man-made
disaster or terrorist attacks. We want Alex to mimic the functionality of a
Search and Rescue robot— which includes remote operation, environment
mapping, and navigation

## System Functionalities

| Name                     | Description                                                                 | Broad Implementation                                                      |
| ------------------------ | --------------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| Remote control/operation | Assuming Alex navigates through unsafe terrains, operation should be remote | Secure shell(SSH) and optionally TCP/IP or Virtual Network Computing(VNC) |
| Environment Mapping      | In order to navigate, Alex requires some awareness of its surroundings      | RPLidar with Hector SLAM                                                  |
| Navigation Controls      | Alex needs to be able to traverse through uneven terrains                   | Timers with wheel encoders/hall sensors                                   |
| Direction signalling     | Alex should have a signal indicating its direction of motion                | Timers with LEDs                                                          |
| Emergency stop           | Alex should have some form of collision prevention                          | Timers with ultrasonic sensors                                            |

## Implementation details

### 1. Remote control/operation

#### 1.1 Remoteness

An operator remotely controls Alex's terminal via an external device, such as a
laptop. This is done with the Secure Shell protocol (SSH) over the network,
which allows us to have remote access to the RPi’s shell, where we can then
run scripts or programs directly from.
As a backup, we can optionally choose to connect via the Virtual Network
Computing(VNC) model, which is a screen sharing system that allows us to
directly control the RPi’s screen. The VNC Model works by a client/server
model, where the server component is installed on the remote computer, in
this case, Alex. The other client, in this case being the operator’s device, will
connect to the server of the RPi, which serves a copy of its screen for the
operator to control.
Another possible backup that we have implemented is to run a TLS Server on
Alex and a TLS Client on our operator’s device to send commands through
TCP/IP

#### 1.2 Reliability

In our current design of Alex, we make use of a mobile hotspot device that
acts as a “WiFi base station”. This mobile hotspot device is placed in between
the remote operator and Alex such that they can both connect to it. This
implies a few things about the reliability of our remote control/operation.
First, it is limited in range. WiFi signals have limited range and this means that
our ability to control Alex remotely is also limited by a range.
Secondly, it is not uncommon for Alex to be rebooted or lose connectivity
temporarily, such that in both cases, Alex disconnects from the common
network temporarily. This is not an issue with our current setup as the IP
address after restoring connection with the mobile hotspot device persists,
and so we simply reconnect after the connection has been restored.
Lastly and most rarely, in the event that our mobile hotspot device fails for
some unknown reason, we will have no means to remotely operate Alex

### 2. Environment Mapping

Environment mapping requires the ability for Alex to locate itself and its
surroundings and be able to describe the shapes of its surroundings as well.
We designed Alex to use a LIDAR unit, which does 360-degree environment
scanning within a 6-meter range

#### 2.1 Reliability

Running visualizations on Alex is expensive- if we were to run a visualization software on Alex itself, it
would take about 94-98% of its CPU power(Alex runs on an RPi 3). Additionally, the visualization
could be laggy and inconsistent and to avoid this issue entirely, we decide to
not run visualizations on Alex.
Instead, by running a ROS Master node on the operator’s remote device, we
send the results of the scans from the LIDAR from the Alex to the
operator’s device which then locally runs a visualization software to create a
360-degree 2D map.

### 3. Navigation controls

At its bare minimum, Alex should be able to:

1\. Move forward or reverse

2\. Turn clockwise or anti-clockwise

3\. Overcome obstacles/uneven terrain

The first two specifications describe how Alex traverses its terrain. Unlike a
car, Alex only either moves forward/reverse or does a turn in a certain
direction. However, it is necessary for such movements must be fully
configurable:

1\. For forward/reverse movements, we should be able to control its speed
and distance to move by.

2\. For turns, we should be able to control the angular speed and angle to
rotate by.

The third specification is necessary for a search and rescue robot that
traverses uneven terrains. However, due to the limited resources of parts that
Alex can be built with, the current design of Alex should minimally be able to:

1\. Overcome humps if necessary

2\. Detect and maneuver away from obstacles

### 1.4 Direction signalling

Similar to a car, we design Alex to have light indicators that show its direction
of motion. In a real world situation, this could be to alert its surroundings to
prevent a collision with living objects.
Alex will have two green LEDs that

1\. Both flash at a fixed interval when it is moving forward, and do not flash
while it reverses

2\. Either one flashes when it is turning i.e. left LED flashes when turning
left while right does not

Similarly, Alex has two red LEDs that:

1\. Both flash at a fixed interval when it is reversing

2\. Either one flashes when it is turning i.e. left LED flashes when turning
left while right does ot

3\. Lastly, Alex will flash both red LEDs when it has completed its search
and rescue operation and parked.

### 1.5 Emergency stop

Alex is built to last. With the use of environment mapping, we are able to
decide how to navigate. However, in the event that the operator sends a
wrong command, if no prevention measure is in place, Alex will collide with an
obstacle.
This leads us to the last function we want in Alex, which is the ability to detect
an obstacle and avoid a collision with. For this we use ultrasonic sensors which are placed
on the front and back of Alex, which will tell us the
proximity of obstacles to the front and back of Alex.
With this, we can program Alex to stop moving at a particular distance away
from an obstacle to prevent collisions.

## System architecture

![System Architecture Overview Image](/assets/SystemArchitecture.jpg)

### 1.1 Description of diagram

Hardware: Yellow/Gold

Software: Blue

### 1.2 Arduino Uno

The Uno runs 4 main modules, which we cover in
detail in Section 4

#### 1.2.1 Serial communication module

The serial communication module helps facilitate the communications
between the Uno and the RPi, for the RPi to relay commands from the remote
operator to the Uno. This module will be used in the main program running on
the Uno, which will then execute instructions to other modules in the Uno
depending on the command

#### 1.2.2 Navigation control module

The motor control module deals with motor controls, and is connected to two
main pieces of hardware - wheel encoders and the L/R motors. The motor
control module helps deal with both speed and distance.
For speed controls, the Uno uses pulse-width modulation(PWM) to control the
motor speed, while for distance control, the Uno is connected to wheel
encoders, which through interrupts, updates global variables required for
calculating distance traveled

#### 1.2.3 Sound detection module

The sound detection module is connected to the Ultrasonic sensors which we
place on the front and back of Alex. This module helps to detect the proximity
of obstacles from Alex, and also helps with collision prevention, where if the
proximity is too close, Alex will stop

#### 1.2.4 Light indication module

The light indication module is connected to 2 Red LEDs and 2 Green Leds,
which are primarily used to indicate the direction of movement of Alex. It uses
timers to periodically flash the leds(similar to a Car’s hazard light), which
makes it more obvious that something is in motion

### 1.3 Raspberry Pi

#### 1.3.1 VNC/SSH/TLS

VNC/SSH/TLS acts as an interface for the operator to send commands to
Alex remotely. To be more specific, we use this interface to run a program on
Alex, which uses the serial communication module to relay the commands to
the Uno. Take note that for any of the interfaces to work, the RPi and the
operator’s device must be connected to the same network

#### 1.3.2 Serial communication module

Similar to the serial communication module in the Uno, we ensure that both
the RPi and the Uno expect the same packet structure. This module acts as a
relay to forward commands from the remote operator to the Uno

#### 1.3.3 rplidar_ros

The rplidar_ros module will be connected to the LIDAR hardware through a
USB connection. This module will be used mainly to run a ROS rplidar node,
which will continuously send the information from the scans of the LIDAR to a
ROS master node

### 1.4 Operator device(Ubuntu 18.04)

#### 1.4.1 VNC/SSH/TLS

VNC/SSH/TLS acts as an interface for the operator to send commands to
Alex remotely. To be specific, both the RPi and the operator’s device must be
connected over the same network for any of the above interfaces to work
properly

#### 1.4.2 ROS

The rplidar_ros module is used in combination with RViz to produce an
understandable visualization of the environment mapping of Alex in real time.
Firstly, roscore is used to run a ros master node. Then, the rplidar_ros module
is used to run a SLAM node on the operator’s device. This SLAM node will
receive scan results from the rplidar node running on the RPi, and then runs
RViz to produce a visualization of the environment mapping.

### 1.5 Power

Alex is powered by two 5V sources: a power bank and a battery pack of size
4\. The power bank is used to power the RPi, which in turn is used to power
the LIDAR and the Uno. The battery pack is used to power the motors that are
connected to the RPi.
The rest of the hardware is connected to/powered by the Uno

## High level algorithm

5 general steps:

1\. Initialization

2\. Movement protocol

3\. Hump/uneven ground protocol

4\. Emergency stop protocol

5\. Parking protocol

### Initialization

- Setting up the “WiFi base station” i.e. mobile hotspot device by powering and enabling the device.
- Turning on power supplies to RPi and motors and operator’s device.
- Connecting the RPi and remote operator device to this common network
- Set up a connection between RPi by using Remote desktop via VNC/TLS/SSH
- Start communication between Arduino and RPi by running main program on RPi and Uno respectively
- Run a ros master node by running `roscore` on the operator's device.
- Launch respective rosnodes on RPi and remote operator’s device
  - RPi: `roslaunch rplidar_ros rplidar.launch`
  - Remote operator: `roslaunch rplidar_ros view_slam.launch`
- All other systems should already be powered
- Scan results of LIDAR are continously fed to the operator device which runs the environment mapping visualization

### Movement protocol

At this stage, we have a visualization of the environment mapping of Alex.

- Observe the mapped LIDAR on RVIZ
- Send appropriate movement command enters “f”, “l”, “r”, “b” to move followed by two extra parameters for “dist” and “speed”
- For our project, we decide to always moved at a fixed distance of 10cm and a fixed speed of 70%, such that it will not be able to move over a bump
- Arduino executes the respective movement
  - Forward/reverse motion - wheels spin the same direction at the same defined speed for a defined distance
  - Left/right turn - wheels turn in opposite direction such that Alex rotates clockwise/anticlockwise at a defined speed for a defined angle
- Depending on the seen visualization, we repeat this until we hit step 6
- Commands that yield no movement imply that we might be in front of a hump, then we initialize the bump protocol.

### Hump/uneven ground protocol

- Check that we are definitely in front of a hump by running a forward command again
- If we are facing a bump, check our visualization to see if we need to explore the area after the bump or not e.g. if it is a parking spot of an empty space with defined walls, we do not go past the wall
- If the bump requires us to explore the area after the bump, then we will move forward 10cm at 90% speed twice, which we have learnt is enough to overcome the hump

### Emergency stop protocol

- In the event the ultrasonic sensor picks up that we are too close to an
  obstacle, it will run stop(), to prevent Alex from collisions
- The default behavior means that stop() is continously being called, which prevents any further movements
- However, Alex should be able to recover from an emergency stop, so we use a global flag `hasStopped` to call stop() if it has not stopped
- Intuitively, this global flag will be reset every time Alex moves forward

### Parking protocol

- In the event we see 3 single unit length walls, we know that is the parking spot.
- We only approach the parking spot after we have mapped out the entire environment

## Hardware design

![Hardware Design Image](/assets/Alex.png)

### Layout

Hardware components were positioned based on priority - on how often they
are accessed and the importance/purpose of the component. The battery pack for
example, is placed at the bottom while the LIDAR, at the top. Another consideration was
weight distribution - we placed LIDAR at the back for even distribution and stability

### Cable management

We managed wiring with the extensive use of cable ties and used
cardboard as padding to ensure that wirings do not move. Since we only had short
cables, we decided to place the breadboard just below the arduino. This would help us
avoid loose connections

### Ultrasonic sensor

Placed at the front of Alex, with the consideration that we will move
Alex forward only(except when turning or recovering from an emergency-stop). The
ultrasonic sensor is used to detect obstacles that are too close to Alex, which will trigger
an auto-stop to prevent collisions, based on our code

## ROS

### What is ROS

Robot Operating System(ROS) is a set of open source software libraries and tools that help you build robot applications
In this context, the important feature is that ROS provides means to run code across multiple computers.

The ROS runtime "graph" is a peer-to-peer network of processes(potentially distributed across machines) that are loosely coupled using the ROS communicatioon infrastructure.

ROS only runs on Unix-based systems for now.

### General concepts/terminologies

- Packages: Main unit for organizing software in ROS - a package may contain ROS runtime processes a.k.a. nodes, a ROS-dependent library, data sets etc. Packages are the most atomic buld item and release item in ROS
- Metapackages: specialized packages which only serve to represent a group of related package
- Package manifests: provides metadata about a package such as its name, versions, description etc
- Repositories: A collection of packages which share a common version control system(VCS).
- Message (msg) types: defines the data structures for messages sent in ROS
- Service (srv) types: define the request and response data structures for services in ROS

### ROS computation graph

The Computation Graph is the peer-to-peer network of ROS processes that are processing data together. Some necessary terminologies are:

- Nodes: processes that perform computation - ROS is designed to be modular i.e. a robot system may contain many nodes that run their own modules. A ROS node is written with the use of a ROS `client` library such as `roscpp` or `rospy`
- Master: The ROS Master provides name registration and lookip to the rest of the CG - without the Master, nodes would not be able to find each other
- Parameter Server: allows data to be stored by key in a central location; is part of Master
- Messages: Nodes communicate with each other by passing `messages`. A message is simply a data structure comprising typed fields. Messages may include nested structures, similar to C structs
- Topics: Messages are routed via a transport system with a **publish/subscribe semantics**. A node sends out a message by `publishing` to a given topic - a name used to identify the content of a message. Another node interested in that particular data will `subscribe` to the appropriate topic. This decouples the production of information from its consumption.
- Services: The publish/subscribe model is a flexible communication paradigm, but not sufficient in a distributed system which often requires request/response interactions. Instead, such interactions are done via `services`, whihc are defined by a pair of message structures - request and response structures. A providing node offers a service under a `name` and a client uses the service by sending the request message and awaiting the reply.
- Bags: Bags are a format for saving and playing back ROS message data. Bags are necessary for developing and testing algorithms.
  Overall, the ROS `master` acts as a nameservice in the ROS CG. It stores `topics` and `services` registration information for ROS `nodes`. Nodes communicate with the `master` to report their registration information and as they communicate with `master`, they can receive information about other registered nodes and make connections as appropriate. Callbacks to these nodes are run when registration information changes, which allows for nodes to dynamically create connections as new nodes are run.

Nodes connect to other nodes directly; We can think of the `master` as a DNS server which only provides lookup information. Nodes that subscribe to a topic will request conections from nodes that publish in that topic and will establish an agreed upon communication protocol. The most common protocol used in ROS is called `TCPROS` which uses standard TCP/IP sockets.

### ROS Community level

The ROS community level concepts are ROs resources that enable separate communities to exchange software and knowledge

- Distributions: ROS Distributions are collections of versioned `stacks` that one can install. Distributions make it easier to install a collection of software with consistent versioning.
- Repositories: ROS relies on a netowrk of code repositories, where different institutions can develop and release their own robot software components
- ROS wiki: The ROS community wiki is the main forum for documenting information about ros. Access it [here](http://wiki.ros.org/)

### ROS in Project Alex

#### 1.1 Installing ROS Melodic https://wiki.ros.org/melodic/Installation/Ubuntu

In Project Alex, we use the ROS Melodic distribution, which is installed on Ubuntu 18.04. These come with a series of tools and libraries that are sufficient for our use case.

#### 1.2 Compiling RPLidar and SLAM ROS packages

- Update package list: `sudo apt-get update`
- Create a catkin workspace: `mkdir -p ~/catkin_ws/src`
- Change directory: `cd ~/catkin_ws/`
- Add `setup.bash` to terminal config: `gedit ~/.bashrc` then append lines `source /opt/ros/melodic/setup.bash` and `source ~/catkin_ws/devel/setup.bash`
- Go to source folder: `cd ~/catin_ws/src`
- Clone RPLidar package: `git clone https://github.com/robopeak/rplidar_ros/tree/slam'
- Go back to root of workspace: `cd ~/catkin_ws/`
- Compile the workspace: `catkin_make`

#### 1.3 Running ROS on RPi and remote operator device

- First, test the connections between our machines by going through mainly step 1 here: http://wiki.ros.org/ROS/NetworkSetup

  - Ensure that both machines are connected over the same network
  - We know that in our use case, we only have 1 publisher and 1 subsciber, so name resolution is not necessary.
  - Instead, we use a fixed local config

    On RPI: `export ROS_IP=<local_ip> && export ROS_MASTER_URI=<master_ip>

    On 18.04: `export ROS_IP=<local_ip> && export ROS_MASTER_URI=<master_ip>

- Next, we test ROS across the two machines following this http://wiki.ros.org/ROS/Tutorials/MultipleMachines
  - Note: we use the above mentioned local config for both machines
- After verifying that our two machines can communicate over ROS, we can simply use the rplidar_ros package which conveniently has two launch files for our needs
- `rplidar.launch`:

  ```
  <launch>
    <node name="rplidarNode"          pkg="rplidar_ros"  type="rplidarNode" output="screen">
    <param name="serial_port"         type="string" value="/dev/ttyUSB0"/>
    <param name="serial_baudrate"     type="int"    value="115200"/>
    <param name="frame_id"            type="string" value="laser"/>
    <param name="inverted"            type="bool"   value="false"/>
    <param name="angle_compensate"    type="bool"   value="true"/>
    </node>
  </launch>
  ```

  - As you can see, `rplidar.launch` runs a single Node(process) with several parameters

- `view_slam.launch`: We edit the default launchfile to remove rplidar.launch since it is run on the RPi

  ```
  <!--
    Used for visualising rplidar in action.

    It requires rplidar.launch.
  -->
  <launch>
    <!-- EDITED LINE
      <include file="$(find rplidar_ros)/launch/rplidar.launch" />
    -->
    <include file="$(find rplidar_ros)/launch/hectormapping.launch" />
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find rplidar_ros)/rviz/slam.rviz" />
  </launch>
  ```

- `hectormapping.launch`

  ```
  <!--
  notice : you should install hector-slam at first,  sudo apt-get install ros-indigo-hector-slam
            this launch just for test, you should improve the param for the best result.

  E-mail: kint.zhao@slamtec.com
  -->
  <launch>

    <node pkg="tf" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 base_link laser 100" /> <!--change -->


      <node pkg="hector_mapping" type="hector_mapping" name="hector_height_mapping" output="screen">
        <param name="scan_topic" value="scan" />
      <param name="base_frame" value="base_link" />
      <param name="odom_frame" value="base_link" />

      <param name="output_timing" value="false"/>
      <param name="advertise_map_service" value="true"/>
      <param name="use_tf_scan_transformation" value="true"/>
      <param name="use_tf_pose_start_estimate" value="false"/>
      <param name="pub_map_odom_transform" value="true"/>
      <param name="map_with_known_poses" value="false"/>

      <param name="map_pub_period" value="0.5"/>
      <param name="update_factor_free" value="0.45"/>

      <param name="map_update_distance_thresh" value="0.02"/>
      <param name="map_update_angle_thresh" value="0.1"/>

      <param name="map_resolution" value="0.05"/>
      <param name="map_size" value="1024"/>
      <param name="map_start_x" value="0.5"/>
      <param name="map_start_y" value="0.5"/>

    </node>
  </launch>
  ```

- As you can see, `view_slam.launch`, which we run on the remote operator device, does most of the work as it runs an `rviz` node i.e. process that runs enviornment mapping visualization, and runs hector SLAM algorithms on the data it receives from the `rplidar` node running on the RPi
