# **R**obot **A**gent **N**ode - RAN

| :books: [Documentation](https://tte-project1.readthedocs.io/en/latest/) |
| --------------------------------------------- |

## Contents
- [**R**obot **A**gent **N**ode - RAN](#robot-agent-node---ran)
  - [Contents](#contents)
  - [Background](#background)
  - [Important](#important)
  - [Prerequisites](#prerequisites)
  - [Install](#install)
  - [Configuration](#configuration)
    - [RAN launch file](#ran-launch-file)
    - [RAN robot description](#ran-robot-description)
  - [License](#license)


## Background

[OPIL](https://opil-documentation.readthedocs.io/) is the Open Platform for Innovations in Logistcs. This platform is meant to enable the development of value added services for the logistics sector in small-scale Industry 4.0 contexts such as those of manufacturing SMEs. In fact, it provides an easy deployable suite of applications for rapid development of complete logistics solutions, including components for task scheduling, path planning, automatic factory layout generation and navigation.

This module is part of the TaskPlanner (TP). TP is one of the three OPIL Components and Functional blocks which this 3rd Layer (mod.sw.tp) is made of. Regarding the OPIL architecture, this node consists of two different sub-modules:

Firstly, the Task Supervisor (mod.sw.tp.ts) monitors the execution of the task dispatched to the agents (Robots). Secondly, the Motion Task Planning (mod.sw.tp.mtp) plans the motion tasks for the robot agents. Task Planner makes it possible for the different components to communicate with each other and be composed into full-fledged logistic system in a manufacturing environment.

## Important

The RAN is currently used in OPIL, but the interface between RAN and TP and also the RAN ROS node are discontinued. The interface will be replaced by the VD(M)A 5050 standard interface for AGVs in version 1.1. The replacement take place in a future TP version.

## Prerequisites
* ROS1 Meldic or Noetic
* (Docker and docker-compose - in case you want to have an easy life ;))
* Understanding defining material flows based on [lotlan](https://lotlan.readthedocs.io/en/latest/)


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

The RAN consists of two configuration files, the launch file which configures the ros node and the robot description which describes the capabilities of the robot platform. 
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

Starts a transformation and a localization for the robot in the simulation. Dont't change this lines 
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

Describes the the type of the robot. This parameter is node further specified or used at the moment. Content is ignored. 

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

This parameter is node further specified or used at the moment.
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

Describes the Vendor. This parameter is node further specified or used at the moment.
```xml
<param name="vendor" value="default"/>
```

This parameter is node further specified or used at the moment.
```xml
<param name="action_capability" value="default"/>
```

Weight of the robot without load.
```xml
<param name="weight" value="1.0"/>
```


## License
[APACHE2](LICENSE) ©