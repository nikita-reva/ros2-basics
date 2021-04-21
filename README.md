# ros2-basics
Boilerplate code for ROS2 applications

---

## Command Line Tools

### General

- Use ROS in terminal  
`source /opt/ros/setup.bash`

- Create a new package  
`ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy`  
`ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp`

- Create new node source code file (my_py_pkg/my_py_pkg)  
`touch my_first_node.py`

- Make file executable  
`chmod +x my_first_node.py`

- Run node  
`./my_first_node.py`

- Edit .bashrc file  
`gedit .bashrc`

- Source .bashrc file  
`source .bashrc`

- View .bashrc file in terminal  
`cat ~/.bashrc`

- Run node from install directory  
`ros2 run my_py_pkg py_node`

- Show all ros2 tools 
`ros2`

- Show all nodes of a package  
`ros2 run my_py_pkg` ((press Tab twice))

- Show ros2 help   
`ros2 -h`

- Start rqt gui  
`rqt`

- Start rqt gui graph  
`rqt_graph`

- Install Turtlesim  
`sudo apt install ros-foxy-turtlesim`

- Launch turtlesim  
`ros2 run turtlesim turtlesim_node`

- Launch turtlesim  
`ros2 run turtlesim turtle_teleop_key`

---

### Build

- Build python package with symlink  
`colcon build --packages-select package_name --symlink-install`

---

### Nodes

- Show node tools  
`ros2 node`

- List all nodes in a graph  
`ros2 node list`

- Show node information  
`ros2 node info /py_test`

- Run node with new name  
`ros2 run package_name node_name --ros-args --remap __node:=new_name`  
`ros2 run package_name node_name --ros-args -r __node:=new_name`

---

### Interfaces

- Create new package for custom interfaces  
`ros2 pkg create interfaces_package_name`  
`cd interfaces_package_name`  
`rm -rf include/ src/`  
`mkdir msg`  
`mkdir srv`  
- Build the interfaces package and view generated file  
`colcon build --packages-select interfaces_package_name`  
`cd ros2_ws/install/interfaces_package_name/lib/python3.8/site-packages/interfaces_package_name/msg`  
`gedit _hardware_status.py`

- Show list of available interfaces in your environment  
`ros2 interface list`

- Show list of available interfaces in a package  
`ros2 interface package sensor_msgs`

- Show interface details  
`ros2 interface show example_interfaces/msg/String`

- Show interface prototype  
`ros2 interface proto example_interfaces/msg/String`

---

### Topics

- Topics functions    
`ros2 topic`

- List all topics  
`ros2 topic list`

- Publish on topic  
`ros2 topic pub /topic_name message_type "{ message_key1: value1, message_key2: value2, ... }"` -r: pub_rate_value

- Echo topic output  
`ros2 topic echo /topic_name`

- Show topic information  
`ros2 topic info /topic_name` 

- Examine topic frequency  
`ros2 topic hz /topic_name`

- Examine topic bandwidth  
`ros2 topic bw /topic_name`

- Publish on topic from terminal  
`ros2 topic pub -r 10 /topic_name example_interfaces/msg/String "{data: 'Hello from Terminal'}"`

- Rename node and topic (publisher)  
`ros2 run my_py_pkg robot_news_station --ros-args -r __node:=my_station -r robot_news:=my_news`

- Rename topic (subscriber)  
`ros2 run my_py_pkg smartphone --ros-args -r robot_news:=my_news`

---

### Services

- Show service options  
`ros2 service`

- Show list of services  
`ros2 service list`

- Show service type (interface -> ros2 interface show /interface.name)  
`ros2 service type /service_name`

- Call service from terminal  
`ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"`

- Call service with new service name (needed in server and clients)  
`ros2 run my_cpp_pkg add_two_ints_server --ros-args -r add_two_ints:=new_name`

---

### Actions

- View action tools  
`ros2 action`

- View actions list  
`ros2 action list`

- View action information  
`ros2 action info /action_name`

- View action information  
`ros2 action send_goal /action_name interface_name "goal_key: goal_value"`

- View action information (with feedback)  
`ros2 action send_goal -f /action_name interface_name "goal_key: goal_value"`

---

### Parameters

- Show list of parameters  
`ros2 param list`

- Get specific parameter of a node  
`ros2 param get /node_name param_name`

- Set specific parameter in terminal  
`ros2 run package_name node_name --ros-args -p param_name:=value`

- Set multiple parameters in terminal  
`ros2 run package_name node_name --ros-args -p param_1_name:=value_1 -p param_2_name:=value_2`

- Set parameter with an array value in terminal  
`ros2 run my_cpp_pkg led_panel --ros-args -p led_states:=[1,1,0]`

---

### Launch files

- Create new launch files package (in ros2_ws/src)  
`ros2 pkg create my_robot_bringup`  
`cd my_robot_bringup`  
`rm -rf include/ src/`  
`mkdir launch`  
`cd lauch`  
`touch number_app.launch.py`  

- Build launch files package  
`colcon build --packages-select my_robot_bringup --symlink-install`

- Start launch file  
`ros2 launch package_name name.launch.py`

--- 

### Bags

- ROS2 bags  
`mkdir bags`  
`cd bags`

- Show bag tools
ros2 bag

- Record output of a topic (stores in a SQLite database)  
`ros2 bag record /topic_name`

- Record output of a topic with a specified file_name  
`ros2 bag record /topic_name -o output_file_name`

- Record output of multiple topics  
`ros2 bag record /topic_1_name /topic_2_name ... /topic_x_name -o output_file_name`

- View bag information  
`ros2 bag info output_file_name`

- Play bag recording  
`ros2 bag play output_file_name`
