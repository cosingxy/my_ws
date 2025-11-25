# 自己的备份程序
## 1.使用需要先编译一下睿尔曼的ROS2包
```bash
//需要编译这三个包，按顺序编译
rm_ros_interfaces
rm_driver
rm_control
```
## 2.还要下载编译moveit，humble版本对应的apt版本moveit或者源码编译都可以
## 3.编译ik求解器
```bash
colcon build --packages-select trac_ik_lib trac_ik_kinematics_plugin
# 只需要编译文件里面的这两个包
```
##  4.然后编译rml_63_ee_description和rml_63_ee_moveit_config
```bash
colcon build --packages-select rml_63_ee_description
source ./install/setup.bash
colcon build --packages-select rml_63_ee_moveit_config
```
