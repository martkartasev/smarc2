# SMaRC2 Packages, Launches and Nodes

This file is auto-generated by [this script](./scripts/render_structure.py) and thus should not be edited by hand!

It is based on [this json](./smarc2_structure.json) which is generated by [this script](./scripts/launch_everything.py).

This list does not contain the common topics that all nodes have(['parameter_events', 'rosout', 'describe_parameters', 'get_parameter_types', 'get_parameters', 'list_parameters', 'set_parameters', 'set_parameters_atomically'])

**SMaRC2 Commit:** f9e046f254bb46ad8eefc92b1bf74cffee56a2f8

**Created:** 1707491975 (2024-02-09 16:19:35)

## [mqtt_bridge](./external/mqtt_bridge)

### [demo.launch.py](./external/mqtt_bridge/launch/demo.launch.py)

> No nodes worked in this launch file!

## [ros2_python_examples](./examples/ros2_python_examples)

### [basic_controller_example.launch](./examples/ros2_python_examples/launch/basic_controller_example.launch)

#### /test_robot/pid_controller_node

##### Subscribers

- **/tf** :: *tf2_msgs/msg/TFMessage*

- **/tf_static** :: *tf2_msgs/msg/TFMessage*

##### Publishers

- **/sam0/core/lcg_cmd** :: *sam_msgs/msg/PercentStamped*

- **/sam0/core/thrust_vector_cmd** :: *sam_msgs/msg/ThrusterAngles*

- **/sam0/core/thruster1_cmd** :: *smarc_msgs/msg/ThrusterRPM*

- **/sam0/core/thruster2_cmd** :: *smarc_msgs/msg/ThrusterRPM*

- **/sam0/core/vbs_cmd** :: *sam_msgs/msg/PercentStamped*

## [ros_tcp_endpoint](./external/ROS-TCP-Endpoint)

### [endpoint.py](./external/ROS-TCP-Endpoint/launch/endpoint.py)

#### /UnityEndpoint

## [sam_basic_controllers](./examples/sam_basic_controllers)

### [sam_control.launch](./examples/sam_basic_controllers/launch/sam_control.launch)

> No nodes worked in this launch file!

## [sam_description](./robot_descriptions/sam_description)

### [sam_description.launch](./robot_descriptions/sam_description/launch/sam_description.launch)

#### /test_robot/joint_state_publisher

##### Subscribers

- **/test_robot/command_states** :: *sensor_msgs/msg/JointState*

- **/test_robot/robot_description** :: *std_msgs/msg/String*

##### Publishers

- **/test_robot/joint_states** :: *sensor_msgs/msg/JointState*

#### /test_robot/robot_state_publisher

##### Subscribers

- **/test_robot/joint_states** :: *sensor_msgs/msg/JointState*

##### Publishers

- **/test_robot/robot_description** :: *std_msgs/msg/String*

- **/tf** :: *tf2_msgs/msg/TFMessage*

- **/tf_static** :: *tf2_msgs/msg/TFMessage*

## [sam_msgs](./messages/sam_msgs)

> This package did not have any launch files.

## [smarc_msgs](./messages/smarc_msgs)

> This package did not have any launch files.

## [smarc_nodered](./gui/smarc_nodered)

### [smarc_nodered.launch](./gui/smarc_nodered/launch/smarc_nodered.launch)

#### /sam/mqtt_bridge_cloud

##### Subscribers

- **/sam/core/gps** :: *sensor_msgs/msg/NavSatFix*
