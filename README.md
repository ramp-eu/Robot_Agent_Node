# **R**obot **A**gent **N**ode - RAN
- [**R**obot **A**gent **N**ode - RAN](#robot-agent-node---ran)
  - [Background](#background)
  - [Important](#important)
  - [Prerequisites](#prerequisites)
  - [Install](#install)
  - [Configuration](#configuration)
    - [RAN launch file](#ran-launch-file)
    - [RAN robot description](#ran-robot-description)
  - [Interfaces](#interfaces)
    - [RAN <-> TP](#ran---tp)
    - [Interfaces consumed by RAN](#interfaces-consumed-by-ran)
      - [MotionAssignment.msg](#motionassignmentmsg)
        - [ROS Message](#ros-message)
      - [CancelTask.msg](#canceltaskmsg)
        - [ROS Message](#ros-message-1)
    - [Interfaces produced by RAN](#interfaces-produced-by-ran)
      - [Motion.msg](#motionmsg)
        - [ROS Message](#ros-message-2)
      - [AssignmentStatus.msg](#assignmentstatusmsg)
        - [ROS Message](#ros-message-3)
      - [RobotAgentDescription.msg](#robotagentdescriptionmsg)
        - [ROS Message](#ros-message-4)
  - [License](#license)
## Background

[OPIL](https://opil-documentation.readthedocs.io/) is the Open Platform for Innovations in Logistcs. This platform is meant to enable the development of value added services for the logistics sector in small-scale Industry 4.0 contexts such as those of manufacturing SMEs. In fact, it provides an easy deployable suite of applications for rapid development of complete logistics solutions, including components for task scheduling, path planning, automatic factory layout generation and navigation.

The RAN (Robot Agent Node) is located between OPIL and the Robot Hardware. It provides two main functionalities: it manages robot navigation, based on ROS, and works as an interface between the robot hardware and the OPIL Cyber Physical Middleware. In order to accomplish the second functionality, the RAN "translates" and adapts Message entities into something understandable by the AGV. Regarding the navigation, a simple PID controller is implemented. It navigates the AGV straight along a line between two received intermediate points by TP. In case a twist is needed, the RAN stops the AGV on a intermediate point, turns the robot in the direction of the next point and proceed with the navigation along the line. 

## Important

The RAN is currently used in OPIL, but the interface between RAN and TP and also the RAN ROS node are discontinued. The interface will be replaced by the VD(M)A 5050 standard interface for AGVs in version 1.1. The replacement take place in a future TP version. 

You can find the deprecated interface msgs in the ros package: **mars_agent_physical_robot_msgs** 

## Prerequisites
* ROS1 Melodic or Noetic
* Further ROS dependencies:  
    * geometry-msgs
    * tf
    * tf2-geometry-msgs
    * angles
    * map-server
    * tf2-eigen
    * eigen-conversions
    * geometry-msgs
    * laser-geometry
    * xacro
    * robot-state-publisher
    * fake-localization 
    * visualization-msgs 
    * amcl
    * laser-geometry 
    * industrial-msgs

## Install
You have to do the following steps:
* Install ROS Melodic or Noetic and create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
```
cd catkin_ws/src
git clone git@github.com:iml130/mod.iot.ran.git
```
* Optional: Install [FIWARE Orion Context Broker](https://fiware-orion.readthedocs.io/en/master/) via [Docker](https://hub.docker.com/r/fiware/orion/)

Important: If you want to use the RAN for testing on the same machine as TP, please delete the following nodes:

* mars_agent_physical_robot_msgs
* mars_common
* mars_common_msgs
* mars_topology_msgs

## Configuration

The RAN (ros node name: mars_simulation_ctv_agent) consists of two configuration files, the launch file which configures the ros node and the robot description which describes the capabilities of the robot platform. 
### RAN launch file

The following parameters are used to configure the RAN node. 

The parameter **robot_1_name** sets the name of the robot inside the RAN ROS node.
```xml
  <arg name="robot_1_name" default="robot_1" />
```

The parameter **initial_pose_robot_1_x** sets the current x-position of the robot in map coordinates.
```xml
  <arg name="initial_pose_robot_1_x" default="-8.916"/>
```

The parameter **initial_pose_robot_1_y** sets the current y-position of the robot in map coordinates.
```xml
  <arg name="initial_pose_robot_1_y" default="-5.12"/>
```

The parameter **initial_pose_robot_1_a** sets the current orientation of the robot.
```xml
  <arg name="initial_pose_robot_1_a" default="0.0"/>
```

The parameter **robot_1_description** sets the robot parameter for the simulation and robot description. The name must be the same as the name of the robot description file (default: "robot_description"). The file ending ".launch" must be ignored!
```xml
  <arg name="robot_1_description" default="robot_description"/>
```

The parameter **robot_1_id** sets the ID of the robot. The ID must be unique and can be random generated or from the robot name. The ID follows the UUID standard (https://en.wikipedia.org/wiki/Universally_unique_identifier). For generating a random UUID the v4 standard must be used, for generating a UUID from a given name UUID standard v5 must be used.
```xml
  <arg name="robot_1_id" default="00000000-0000-0000-0000-000000000001"/>
```

This line is necessary to start firos and communicate with orion. DON'T CHANGE THIS LINE!
```xml
  <!-- ****** Firos ***** -->
  <node name="firos" pkg="firos" type="core.py"/>
```

This line starts rviz which displays the current position of the robot. If no visualization is favoured, comment this line out.
```xml  
  <!-- RVIZ -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find mars_simulation)/rviz/opil_finnland.rviz" />
```

This line starts the build in simulation.
```xml
  <!--  ****** Stage simulation *****  -->
  <include file="$(find mars_simulation)/launch/opil_finnland_simulation.launch"/>
```
 Sets the namespace for the runs node(s). (In v3.x no namespace is used)
```xml
  <!-- <group ns="/opil/iot"> -->
  .. some content ...
  <!-- </group> -->
```

Starts the RAN node for one robot. Each robot needs an individual RAN. All needed parameter are configured via the arguments above (**arg**).
```xml
  <include file="$(find mars_simulation_ctv_agent)/launch/mars_simulation_ctv_agent.launch">
  .. allot of parameter ...
  </include>
```

Starts a transformation and a localization for the robot in the simulation. Don't change this lines 
```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="0 0 0 0 0 0 1 map robot_1/odom" />

      <node name="fake_localization" pkg="fake_localization" type="fake_localization">
        <param name="odom_frame_id" value="robot_1/odom"/>
        <param name="base_frame_id" value="robot_1/base_link"/>
      </node>

      <!--  ***************** Robot Model *****************  -->
      <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find mars_simulation)/urdf/ctv_1.xacro'" />
      <node pkg="robot_state_publisher" type="state_publisher" name="robot_1_state_publisher" />
```

### RAN robot description

The following parameters are used to configure the RAN node. The parameters are stored inside a robot description launch file which will be include in the RAN launch file.

Describes the the type of the robot. This parameter is not further specified or used at the moment. Content is ignored. 

```xml
<param name="type" value="default"/>
```

Describes the value of the proportional term of the PID controller inside the RAN. Term is configured for the example simulation. Term must be individually determined for each AGV.

```xml
<param name="gain_kp" value="1.5"/>
```
Describes the value of the integral term of the PID controller inside the RAN. Term is configured for the example simulation. Term must be individually determined for each AGV.

```xml
<param name="gain_ki" value="1.0"/>
```
Describes the value of the derivative term of the PID controller inside the RAN. Term is configured for the example simulation. Term must be individually determined for each AGV.

```xml
<param name="gain_kd" value="0.0001"/>
```

Description of the footprint of an AGV as a polygon. Only the X value is set here. In the case of specifying the footprint, the center of the robot is assumed to be at (0.0, 0.0) and both clockwise and counterclockwise specifications are supported. Footprint is described in meter.  **IMPORTANT: The turning radius, longest site of the footprint, must be smaller then the "mars_vertex_footprint_radius" in TP (mod.sw.tp). (turning_radius < mars_vertex_footprint_radius)**

```xml
<rosparam param="footprint_x">[0.35, -0.35, -0.35, 0.35]</rosparam>
```

Description of the footprint of an AGV as a polygon. Only the Y value is set here. In the case of specifying the footprint, the center of the robot is assumed to be at (0.0, 0.0) and both clockwise and counterclockwise specifications are supported. Footprint is described in meter. **IMPORTANT: The turning radius, longest site of the footprint, must be smaller then the "mars_vertex_footprint_radius" in TP (mod.sw.tp). (turning_radius < mars_vertex_footprint_radius)**
```xml
<rosparam param="footprint_y">[0.25, 0.25, -0.25, -0.25]</rosparam>
```

Minimal height in meter of the robot without the load.
```xml
<param name="min_height" value="0.383"/>
```
Maximal height in meter of the robot with load on top.
```xml
<param name="max_height" value="0.383"/>
```

Payload which the robot can carry in kilogram.
```xml
<param name="payload" value="1.0"/>
```

Maximum velocity of the robot in m/s in forward direction. 
```xml
<param name="max_pos_x_vel" value="1.1"/>
```

Maximum velocity of the robot in m/s in reverse direction. 
```xml
<param name="max_neg_x_vel" value="1.1"/>
```

Maximum acceleration of the robot in forward direction in m/s².
```xml
<param name="max_pos_x_acc" value="0.5"/>
```

Maximum brake acceleration of the robot in forward direction in m/s².
```xml
<param name="max_neg_x_acc" value="0.5"/>
```

Maximum velocity of the robot in m/s in left direction. 
```xml
<param name="max_pos_y_vel" value="0.0"/>
```

Maximum velocity of the robot in m/s in right direction. 
```xml
<param name="max_neg_y_vel" value="0.0"/>
```

Maximum acceleration of the robot in side direction in m/s².
```xml
<param name="max_pos_y_acc" value="0.0"/>
```

Maximum brake acceleration of the robot in side direction in m/s².
```xml
<param name="max_neg_y_acc" value="0.0"/>
```

Maximum angular velocity of the robot in rad/s. 
```xml
<param name="max_pos_ang_vel" value="1.1"/>
```

Maximum negative angular velocity of the robot in rad/s. 
```xml
<param name="max_neg_ang_vel" value="1.1"/>
```

Maximum angular acceleration of the robot in rad/s².
```xml
<param name="max_pos_ang_acc" value="0.5"/>
```

Maximum angular brake acceleration of the robot in rad/s².
```xml
<param name="max_neg_ang_acc" value="0.5"/>
```

This parameter is not further specified or used at the moment.
```xml
<param name="velocity_control_sensitivity" value="1.0"/>
```

Describes the type of drive. A differential drive is described by zero turing radius. Ackermann steering by its minimum turning radius. Only differential drive is supported by TP modul (mod.sw.tp).   
```xml
<param name="min_turning_radius" value="0.0"/>
```

Describes the battery capacity in Amp-hr (Ah). Parameter not used.
```xml
<param name="batt_capacity" value="1.0"/>
```

Describes the battery max voltage in volt (V). Parameter not used.
```xml
<param name="batt_max_voltage" value="1.0"/>
```

Describes the vendor. This parameter is not further specified or used at the moment.
```xml
<param name="vendor" value="default"/>
```

This parameter is not further specified or used at the moment.
```xml
<param name="action_capability" value="default"/>
```

Weight of the robot without load.
```xml
<param name="weight" value="1.0"/>
```

## Interfaces

Subsequently a detailed description of the exchanged entites is given. 

### RAN <-> TP

The following listing depicts the complete exchanged messages between RAN and TP. Subsequently all key value pairs will be explained in detail.

```json
{
    "id": "robot_opil_v2",
    "type": "ROBOT",
    "action_assignment": {
     ... },
    "cancel_order": {
     ... },
    "current_motion": {
     ... },
    "motion_assignment": {
     ... },
    "robot_description": {
     ... }
}
```

### Interfaces consumed by RAN

In the following all messages will be explained which are send by the TP. Messages might be based on some non-primitive types (e.g. mars_common_msgs/Id and others); These types are explained at the end of this document ([Used messages inside TP messages](#used-messages-inside-tp-messages)).

#### MotionAssignment.msg

The motion assignment tells the AGV the next destination and under which circumstances it can moves to this position.

| Type                         | Variable         | Description                                                                                                                                                                                                       |
| ---------------------------- | ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| mars_common_msgs/Id          | point_id         | ID of the next vertex / edge were the AGV should drive to.                                                                                                                                                        |
| mars_common_msgs/Id          | task_id          | ID of the task to which the MotionAssignment belongs.                                                                                                                                                             |
| mars_common_msgs/Id          | motion_id        | ID of the MotionAssignment. A new ID must be generated for each MotionAssignment.                                                                                                                                 |
| bool                         | is_waypoint      | TRUE if the point is a waypoint (intermediate point along the path), FALSE if it is a goal.                                                                                                                       |
| bool                         | use_orientation  | TRUE if the theta of the point has to be considered.                                                                                                                                                              |
| geometry_msgs/Twist          | max_velocity     | Maximum allowed velocity in the current segment. Segment is defined by the **motion_area**. (For more information about the message visit: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)            |
| geometry_msgs/Accel          | max_acceleration | Maximum allowed acceleration in the current segment. Segment is defined by the **motion_area**. (For more information about the message visit: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Accel.html) |
| geometry_msgs/PolygonStamped | motion_area      | Area in which the vehicle can move freely. (For more information about the message visit: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PolygonStamped.html)                                             |
| Sequence                     | sequence         | Sequence number of the current MotionAssignment.                                                                                                                                                                  |

##### ROS Message
```ini
Header header
mars_common_msgs/Id point_id
mars_common_msgs/Id task_id
mars_common_msgs/Id motion_id
geometry_msgs/Pose2D point
# TRUE if the point is a waypoint, FALSE if it is a goal
bool is_waypoint
# TRUE if the theta of the point has to be considered
bool use_orientation
geometry_msgs/Twist max_velocity
geometry_msgs/Accel max_acceleration
# defines the area in which the robot can move
geometry_msgs/PolygonStamped motion_area
# the position of the assignment in the sequence
Sequence sequence
```

#### CancelTask.msg

The cancel task message cancels a whole task for an AGV.

| Type                | Variable  | Description                                                                                                                 |
| ------------------- | --------- | --------------------------------------------------------------------------------------------------------------------------- |
| mars_common_msgs/Id | task_id   | ID of the task which should be canceled. If an Action- or MotionID is additionally given, only this part will be cancelled. |
| mars_common_msgs/Id | action_id | NOT supported at the moment!                                                                                                |
| mars_common_msgs/Id | motion_id | Not supported at the moment!                                                                                                |

##### ROS Message

```ini
# task ID instead of action id because the message deletes the whole task
# the task is a sequence of motions and actions
mars_common_msgs/Id task_id
mars_common_msgs/Id action_id
mars_common_msgs/Id motion_id
```

### Interfaces produced by RAN

#### Motion.msg

The motion message is a combination of the current position of the AGV in the global coordinate system provided by S&P and the current velocity. 

| Type                      | Variable         | Description                                                                                                                                          |
| ------------------------- | ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| geometry_msgs/PoseStamped | current_position | Current position of the AGV. (For more information about the message visit: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) |
| geometry_msgs/Twist       | current_velocity | Current velocity of the AGV. (For more information about the message visit: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)              |

##### ROS Message

```ini
geometry_msgs/PoseStamped current_position
geometry_msgs/Twist current_velocity
```

#### AssignmentStatus.msg

The assignment status gives you an overview which task is currently executed by the AGV and which was the last finished task.

| Type                         | Variable             | Description                                           |
| ---------------------------- | -------------------- | ----------------------------------------------------- |
| mars_common_msgs/Id          | current_task_id      | Id of the current task which is executed.             |
| mars_common_msgs/Id          | current_motion_id    | Id of the current MotionAssignment which is executed. |
| mars_common_msgs/Id          | current_action_id    | Id of the current ActionAssignment which is executed. |
| mars_common_msgs/Id          | last_finished_motion | Id of the last finished MotionAssignment.             |
| mars_common_msgs/Id          | last_finished_action | Id of the last finished ActionAssignment.             |
| geometry_msgs/PolygonStamped | footprint            | Current footprint of the AGV, including load.         |

##### ROS Message

```ini
mars_common_msgs/Id current_task_id
mars_common_msgs/Id current_motion_id
mars_common_msgs/Id current_action_id
mars_common_msgs/Id last_finished_motion
mars_common_msgs/Id last_finished_action
geometry_msgs/PolygonStamped footprint
```

#### RobotAgentDescription.msg

This message describes the AGVs footprint, kinematic and the capabilities like lifting operations. 

| Type                         | Variable          | Description                                                                                                                                                                                                                                        |
| ---------------------------- | ----------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| mars_common_msgs/Id          | robot_id          | ID of the robot                                                                                                                                                                                                                                    |
| VehicleType                  | type              | Defines the type of the AGV                                                                                                                                                                                                                        |
| geometry_msgs/PolygonStamped | footprint         | The footprint is the contour of the mobile base. In ROS, it is a two dimensional array of the form [x0, y0],[x1, y1], ..., [xn, yn]]. The origin of the coordinates should be the center of the robot (center of rotation for differential drive). |
| float32                      | min_height        | Minimal height of the AGV in meter.                                                                                                                                                                                                                |
| float32                      | max_height        | Maximal height of the AGV in meter.                                                                                                                                                                                                                |
| float32                      | payload           | Maximum Payload which can be carried by the AGV in kilogram.                                                                                                                                                                                       |
| float32                      | max_pos_x_vel     | Maximum positive velocity in driving direction in m/s.                                                                                                                                                                                             |
| float32                      | max_neg_x_vel     | Maximum negative speed in reverse direction in m/s.                                                                                                                                                                                                |
| float32                      | max_pos_x_acc     | Maximum positive acceleration in driving direction in m/s².                                                                                                                                                                                        |
| float32                      | max_neg_x_acc     | Maximum negative acceleration in driving direction in m/s².                                                                                                                                                                                        |
| float32                      | max_pos_y_vel     | Maximum positive velocity in y direction for omnidirectional AGV in m/s.                                                                                                                                                                           |
| float32                      | max_neg_y_vel     | Maximum negative velocity in y direction for omnidirectional AGV in m/s.                                                                                                                                                                           |
| float32                      | max_pos_y_acc     | Maximum positive acceleration in y direction for omnidirectional AGV in m/s².                                                                                                                                                                      |
| float32                      | max_neg_y_acc     | Maximum negative acceleration in y direction for omnidirectional AGV in m/s².                                                                                                                                                                      |
| float32                      | max_pos_ang_v     | Maximum positive angular velocity in m/s.                                                                                                                                                                                                          |
| float32                      | max_neg_ang_v     | Maximum negative angular velocity in m/s.                                                                                                                                                                                                          |
| float32                      | max_pos_ang_a     | Maximum positive angular acceleration in m/s².                                                                                                                                                                                                     |
| float32                      | max_neg_ang_a     | Maximum negative angular acceleration in m/s².                                                                                                                                                                                                     |
| float32                      | velocity_cont     | ???                                                                                                                                                                                                                                                |
| float32                      | min_turning_r     | Turning radius in meter. For differential drives it is zero!                                                                                                                                                                                       |
| float32                      | batt_capacity     | Maximum capacity of the battery in Ah.                                                                                                                                                                                                             |
| float32                      | batt_max_volt     | Maximum voltage of the battery in V.                                                                                                                                                                                                               |
| float32                      | weight            | Weight of the AGV in kg.                                                                                                                                                                                                                           |
| string                       | vendor            | Vendor of the AGV.                                                                                                                                                                                                                                 |
| RobotAction []               | action_capability | A list of Actions which can be performed by the AGV.                                                                                                                                                                                               |

##### ROS Message

```ini
mars_common_msgs/Id robot_id
VehicleType type
geometry_msgs/PolygonStamped footprint
float32 min_height
float32 max_height
float32 payload
float32 max_pos_x_vel
float32 max_neg_x_vel
float32 max_pos_x_acc
float32 max_neg_x_acc
float32 max_pos_y_vel
float32 max_neg_y_vel
float32 max_pos_y_acc
float32 max_neg_y_acc
float32 max_pos_ang_vel
float32 max_neg_ang_vel
float32 max_pos_ang_acc
float32 max_neg_ang_acc
float32 velocity_control_sensitivity
float32 min_turning_radius
float32 batt_capacity
float32 batt_max_voltage
float32 weight
string vendor 
RobotAction[] action_capability

```

## License
[APACHE2](LICENSE) ©