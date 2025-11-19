"""
ROS 2 集成测试启动文件 - Hybrid Planning 基本功能测试

作用：
1. 启动完整的 Hybrid Planning 系统（全局规划器、局部规划器、管理器）
2. 运行 C++ Google Test 测试程序（test_basic_integration.cpp）
3. 验证测试是否成功完成

测试流程：
1. 启动 Hybrid Planning 组件
2. 等待 2 秒让系统稳定
3. 运行测试程序（发送目标、验证执行结果）
4. 检查测试退出码（0 表示成功）
"""
import launch_testing
import os
import sys
import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# 将当前目录添加到 Python 路径，以便导入 hybrid_planning_common.py
sys.path.append(os.path.dirname(__file__))
from hybrid_planning_common import (
    generate_common_hybrid_launch_description,  # 生成 Hybrid Planning 组件
    get_robot_description,                       # 获取 URDF
    get_robot_description_semantic,              # 获取 SRDF
    load_file,                                   # 加载文件工具函数
    load_yaml,                                   # 加载 YAML 工具函数
)


def generate_test_description():
    """
    生成测试启动描述
    
    步骤：
    1. 启动 Hybrid Planning 基础组件（从 hybrid_planning_common.py）
    2. 加载机器人描述（URDF 和 SRDF）
    3. 加载 Hybrid Planning 参数
    4. 创建测试节点（C++ Google Test 可执行文件）
    5. 延迟 2 秒后启动测试（等待组件完全启动）
    """
    # 步骤1: 启动 Hybrid Planning 组件
    # 包含：GlobalPlannerComponent, LocalPlannerComponent, HybridPlanningManager
    common_launch = generate_common_hybrid_launch_description()
    
    # 步骤2: 获取机器人描述
    # URDF: /home/gxy/my_ws/install/rml_63_ee_moveit_config/share/rml_63_ee_moveit_config/config/rml_63_ee_description.urdf.xacro
    robot_description = get_robot_description()
    
    # SRDF: /home/gxy/my_ws/install/rml_63_ee_moveit_config/share/rml_63_ee_moveit_config/config/rml_63_ee_description.srdf
    robot_description_semantic = get_robot_description_semantic()

    # 步骤3: 加载 Hybrid Planning 参数
    # 文件路径: /home/gxy/my_ws/install/rm63_ee_hybrid_planning/share/rm63_ee_hybrid_planning/config/common_hybrid_planning_params.yaml
    common_hybrid_planning_param = load_yaml(
        "rm63_ee_hybrid_planning", "config/common_hybrid_planning_params.yaml"
    )

    # 步骤4: 创建测试节点
    # 可执行文件位置: /home/gxy/my_ws/build/rm63_ee_hybrid_planning/test_basic_integration
    # 这是 test_basic_integration.cpp 编译后的可执行文件
    hybrid_planning_gtest = Node(
        executable=PathJoinSubstitution(
            [LaunchConfiguration("test_binary_dir"), "test_basic_integration"]
        ),
        parameters=[
            robot_description,              # 传入机器人 URDF
            robot_description_semantic,     # 传入机器人 SRDF
            common_hybrid_planning_param,   # 传入 Hybrid Planning 参数
        ],
        output="screen",  # 将测试输出打印到终端
    )

    # 步骤5: 返回启动描述
    return LaunchDescription(
        [
            # 声明参数：测试二进制文件目录（由 colcon test 自动传入）
            DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            # 延迟 2 秒启动测试（等待 Hybrid Planning 组件完全启动）
            TimerAction(period=2.0, actions=[hybrid_planning_gtest]),
            # 标记测试准备就绪
            launch_testing.actions.ReadyToTest(),
        ]
        + common_launch  # 添加 Hybrid Planning 组件
    ), {
        # 导出测试节点句柄，供测试用例使用
        "hybrid_planning_gtest": hybrid_planning_gtest,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    """
    测试用例1：等待测试完成
    
    作用：
    - 等待 Google Test 可执行文件运行完成
    - 超时时间：4000 秒（约 66 分钟）
    - 如果超时，测试失败
    """
    def test_gtest_run_complete(self, hybrid_planning_gtest):
        self.proc_info.assertWaitForShutdown(hybrid_planning_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    """
    测试用例2：验证测试退出码（在所有进程关闭后运行）
    
    作用：
    - 检查测试进程的退出码
    - 退出码 0 = 成功，非 0 = 失败
    - Google Test 会根据测试结果返回对应退出码
    """
    def test_gtest_pass(self, proc_info, hybrid_planning_gtest):
        launch_testing.asserts.assertExitCodes(proc_info, process=hybrid_planning_gtest)
