# ROS [ros_control](http://wiki.ros.org/ros_control) wiki 学习笔记



本文为笔者根据 ros_control wiki 整理的笔记，文中大多为对原文的翻译，主要用于作者后续查阅。读者在阅读之前可根据自身情况决定是否阅读。



## 1 架构

![gazebo_ros_control](.readme/pic_resource/gazebo_ros_control.png)

ros_control 将来自机器人执行器编码器的关节状态数据和输入设定点作为输入。它使用一种通用的控制回路反馈机制（通常是 PID 控制器）来控制发送到执行器的输出（通常是作用力）。对于没有一对一映射的关节位置和力的物理机构，ros_control 变得更加复杂，但这些场景是使用传输来解释的（Transmissions）。





## 2 控制器

可用的控制器插件

- joint_state_controller（将注册到 hardware_interface::JointStateInterface 的所有资源状态发布到 sensor_msgs/JointState 类型的话题）
  - joint_state_controller
- position_controllers（对硬件控制期望位置）
  - joint_position_controller（接收位置输入并发送位置输出，只需使用 [forward_command_controller](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/forward_command_controller) 控制器传输输入即可。）
  - joint_group_position_controller（同时设置多关节位置）
- velocity_controllers（对硬件控制期望速度）
  - joint_position_controller（使用PID控制器接收位置输入并发送速度输出，）
  - joint_velocity_controller（接收速度输入并发送速度输出，只需使用 [forward_command_controller](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/forward_command_controller) 控制器传输输入即可。）
  - joint_group_velocity_controller（同时设置多关节速度）
- effort_controllers（对硬件控制期望作用（力/力矩））
  - joint_position_controller（使用PID控制器接收位置输入并发送作用力输出。）
  - joint_group_position_controller（同时设置多关节位置）
  - joint_velocity_controller（使用PID控制器接收速度输入并发送作用力输出。）
  - joint_effort_controller（接收作用力输入并发送作用力输出，只需使用 [forward_command_controller](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/forward_command_controller) 控制器传输输入即可。）
  - joint_group_effort_controller（同时设置多关节作用力）
- joint_trajectory_controllers（样条连接整条轨迹实现额外功能）（[src](https://github.com/ros-controls/ros_controllers/blob/melodic-devel/joint_trajectory_controller/src/joint_trajectory_controller.cpp)）
  - position_controller
  - velocity_controller
  - effort_controller
  - position_velocity_controller
  - position_velocity_acceleration_controller







## 3 硬件接口

ROS控制与上述一个ROS控制器一起使用硬件接口向硬件发送和接收命令。撰写本文时可用硬件接口的列表（通过硬件资源管理器）。如果您的机器人的硬件接口尚不存在，您当然可以创建自己的接口，但不限于此列表：

- Joint Command Interface（）
  - Effort Joint Interface（用于控制基于作用力的关节）
  - Velocity Joint Interface（用于控制基于速度的关节）
  - Position Joint Interface（用于控制基于位置的关节）
- Joint State Interfaces
- Actuator State Interface
- PosVelJoint Interface
- PosVelAccJoint Interface
- Force-torque sensor Interface
- IMU sensor Interface





## 4 Transmissions

Transmissions是控制管道中的一个元素，用于转换作用力/流量变量，使其产生-功率-保持不变。

机械传诵是保持功率的变换

```
P_in        = P_out
F_in x V_in = F_out x V_out
```

上述 P、F、V 分别表示功率、力和速度。一般来说，功率是作用力（如力、电压）和流量（如速度、电流）变量的乘积。对于减速比为n的简单机械减速器，有：

```
effort map: F_joint = F_actuator * n
flow map:   V_joint = V_actuator / n
```

从上面可以看出，功率在输入和输出之间保持不变。（https://en.wikipedia.org/wiki/Bond_graph）



### 4.1 Transmission URDF Format

[URDF Transmission](https://wiki.ros.org/urdf/XML/Transmission)



### 4.2 Transmission Interface

Transmission专用代码（非机器人专用），在 Transmission 类型共享的统一接口下实现双向（执行器<->关节）作用力和flow。

Transimission 类型

- Simple Reduction Transmission
- Differential Transmission
- Four Bar Linkage Transimission

用途

- transmission_interface::[ActuatorToJointStateInterface ](http://wiki.ros.org/ActuatorToJointStateInterface)从执行器变量填充关节状态。
- hardware_interface::[JointStateInterface](http://wiki.ros.org/JointStateInterface) 向控制器公开关节状态。



### 4.3 [Transmission Example](https://github.com/ros-controls/ros_control/wiki/transmission_interface)





## 5 关节限制

 joint_limits_interface 包含用于表示关节限制的数据结构、从通用格式（如URDF和ROSPARM）填充关节限制的方法，以及在不同类型的关节命令上强制执行限制的方法。

 joint_limits_interface 不是由控制器本身使用的（它不实现硬件接口），而是在控制器更新后，在robot抽象的write（）方法（或等效方法）中运行。

强制限制将覆盖控制器设置的命令，它不会在单独的原始数据缓冲区上运行。



### 5.1 规范

- Joint limits 位置、速度、加速度、jerk和作用力。
- Soft joint limits 软位置限制，k_ p，k_ v（example: [pr2_controller_manager/safety_limits]( pr2_controller_manager/safety_limits)）
- 从URDF加载关节限制信息的实用方法（仅位置、速度、作用力）。
- 从URDF加载软接头极限信息的实用方法。
- 从ROS参数服务器加载关节限制的实用方法（所有值）。参数规范与MoveIt中使用的参数规范相同，此外，还分析jerk和effort限制。



### 5.2 关节限制接口

- ros_control 接口用于强制执行关节限制
- 对于 *effort-controlled* joints, *position-controlled* joints, and *velocity-controlled* joints，创建了两种类型的接口。第一种是饱和界面，用于具有正常限制但不具有软限制的关节。第二个是实现软限制的接口，类似于PR2上使用的接口。



### 5.3 [Joint limits Example](https://github.com/ros-controls/ros_control/wiki/joint_limits_interface)



## 6 其他示例

- Barrett WAM controllers at Johns Hopkins University: [barrett_control on Github](https://github.com/jhu-lcsr/barrett_control)
- RRBot in Gazebo: [ros_control with Gazebo tutorial](http://gazebosim.org/tutorials/?tut=ros_control)
- Rethink Baxter hardware as used at the University of Colorado Boulder: [Baxter on Github](https://github.com/davetcoleman/baxter_ssh)
- ROS HAL interface provides interfaces to [MachineKit](http://wiki.ros.org/MachineKit) HAL from ROS: https://github.com/zultron/hal_ros_control/



## 7 安装

二进制包安装

```
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```

源代码安装

```
cd CATKIN_WORKSPACE/src
wstool init
wstool merge https://raw.github.com/ros-controls/ros_control/melodic-devel/ros_control.rosinstall
wstool update
cd ..
rosdep install --from-paths . --ignore-src --rosdistro melodic -y
catkin_make
```





## Reference

[ros_control GitHub Wiki](https://github.com/ros-controls/ros_control/wiki)

[ROScon 2014 talk entitled ros_control Video](https://vimeo.com/107507546)

[ROScon 2016 talk OmbineRobotHW Video](https://vimeo.com/187696094)

[ros-urdf-xml](https://wiki.ros.org/urdf/XML)