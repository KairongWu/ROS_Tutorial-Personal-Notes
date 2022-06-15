安装与环境配置

检查环境变量

```
printenv | grep ROS
```



初始化终端ROS环境

```
source /opt/ros/<distro>/setup.bash
source /opt/ros/melodic/setup.bash
```



[catkin_or_rosbuild](https://wiki.ros.org/action/fullsearch/catkin_or_rosbuild?action=fullsearch&context=180&value=linkto%3A"catkin_or_rosbuild")

[catkin](https://wiki.ros.org/catkin)

[workspaces](https://wiki.ros.org/action/fullsearch/catkin/workspaces?action=fullsearch&context=180&value=linkto%3A"catkin%2Fworkspaces")



创建 ROS 工作空间

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make		# 首次运行将在 src 文件夹目录下创建 CMakeLists.txt
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3	# 对于 python3 用户需要使用该命令编译
source devel/setup.bash
echo $ROS_PACKAGE_PATH # 检查当前终端的环境变量是否初始化成功
```







文件系统

编译工具 catkin rosbuild

1. [catkin](https://wiki.ros.org/catkin)
2. [catkin/cinceptual_overview](catkin/cinceptual_overview)
3. [catkin_or_rosbuild](https://wiki.ros.org/catkin_or_rosbuild)
4. [rosbuild](https://wiki.ros.org/rosbuild)



命令行工具

- roscd
- rosls
- rospack



```
sudo apt-get install ros-<distro>-ros-tutorials
sudo apt-get install ros-melodic-ros-tutorials
```



文件系统概念

- 功能包(packages)：功能包是 ROS 代码软件组成的最小单元。每一个功能包都包含库、可执行程序，脚本和其他
- 清单(package.xml)：清单是对包的描述。用于定义包之间的依赖关系，并捕获有关包的元信息，如版本、维护者、许可证等



文件系统工具

rospack 用于获取功能包信息，如获取功能包的路径

```
rospack find [ package_name ]
rospack find roscpp
YOUR_INSTALL_PATH/share/roscpp
/opt/ros/kinetic/share/roscpp
```



roscd roscd是rosbash套件的一部分。用于切换目录至目标功能包

```
roscd <package-or-stack>[/subdir]
roscd roscpp
pwd
YOUR_INSTALL_PATH/share/roscpp
```

请注意，与其他ROS工具一样，roscd只能找到[ROS_PACKAGE_PATH](https://wiki.ros.org/ROS/EnvironmentVariables#ROS_PACKAGE_PATH)路径中列出的目录中的ROS包。要查看[ROS_PACKAGE_PATH](https://wiki.ros.org/ROS/EnvironmentVariables#ROS_PACKAGE_PATH)路径中的内容，请键入：

```
 echo $ROS_PACKAGE_PATH
```



移动至子目录

```
roscd roscpp/cmake
pwd
YOUR_INSTALL_PATH/share/roscpp/cmake
```





roscd log 将进入ROS存储日志文件的文件夹。请注意，如果尚未运行任何ROS程序，这将产生一个错误，说明它尚不存在。

```
roslog
```





rosls 是rosbash工具的一部分。它允许你按名称而不是按绝对路径直接在包中ls。

```
rosls <package-or-stack>[/subdir]
rosls roscpp_tutorials
cmake launch package.xml  srv
```





Tab 智能补充

```
roscd roscpp_tut<<< now push the TAB key >>>
roscd roscpp_tutorials/	# 按 TAB ，命令行自动填充
```



















