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
