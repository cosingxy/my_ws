#!/bin/bash

# RML 63 EE MoveIt 启动脚本

echo "======================================"
echo "  启动 RML 63 EE MoveIt 系统"
echo "======================================"
echo ""

# 1. Source 必要的工作空间
echo "📦 加载工作空间..."
source /home/gxy/ros2_ws/install/setup.bash
source /home/gxy/ws_moveit2/install/setup.bash
source /home/gxy/my_ws/install/setup.bash

echo "✅ 工作空间已加载"
echo ""

# 2. 检查包是否存在
echo "🔍 检查 rml_63_ee_moveit_config 包..."
if ros2 pkg prefix rml_63_ee_moveit_config &> /dev/null; then
    echo "✅ rml_63_ee_moveit_config 包已找到"
else
    echo "❌ rml_63_ee_moveit_config 包未找到！"
    echo "   请先编译: colcon build --packages-select rml_63_ee_moveit_config"
    exit 1
fi

echo ""
echo "🚀 启动 MoveIt (real.launch.py)..."
echo "   - robot_state_publisher: 发布机器人模型"
echo "   - move_group: 规划和执行服务"
echo "   - RViz: 可视化界面"
echo ""
echo "⚠️  注意：确保机器人驱动器已启动并发布 /joint_states"
echo ""

# 3. 启动 launch 文件
ros2 launch rml_63_ee_moveit_config bringup.launch.py
