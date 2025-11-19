from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("rml_63_ee_description",package_name="rml_63_ee_moveit_config" ).to_moveit_configs())
    ld = LaunchDescription()
    # 2. 静态TF：world -> base_link
    #    RViz需要这个才能把场景摆在world坐标系下
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="worldTobase_TF",
        arguments=[
            "0", "0", "0",           # translation (x y z)
            "0", "0", "0",           # rotation RPY (roll pitch yaw)
            "world",                 # parent frame
            "base_link",             # child frame
        ],
        output="screen",
    )
    ld.add_action(static_tf_node)
    # 静态TF，从base_link到camera_link
    static_camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="baseToCamera1_TF",
        arguments=[
            "-0.0258", "0.1555", "0.075",
            "0", "0", "1.57",
            "base_link",
            "camera_link",
        ],
        output="screen",
    )
    ld.add_action(static_camera_tf_node)

    static_camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="EeToCamera2_TF",
        arguments=[
            "-0.0960266", "0.04024", "0.04578",
            "-0.02568", "-0.01020", "-1.59852",
            # 第一个是绕蓝色Z轴旋转，其次是绕绿色Y轴，最后是绕红色X轴，逆时针为正
            "link6",
            "Scepter_frame",
        ],
        output="screen",
    )
    ld.add_action(static_camera_tf_node)


    static_camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ScepterFrameToScepterDepthFrame_TF",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            # 第一个是绕蓝色Z轴旋转，其次是绕绿色Y轴，最后是绕红色X轴，逆时针为正
            "Scepter_frame",
            "Scepter_depth_frame",
        ],
        output="screen",
    )
    ld.add_action(static_camera_tf_node)


    # 3. robot_state_publisher
    #    发布robot_description到TF，用的是你现在这个带ee_link的URDF
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    ld.add_action(robot_state_pub_node)

    # 4. move_group 节点（规划 + 执行接口）
    add_move_group(ld, moveit_config)

    # 5. RViz2 节点（可视化 + 交互Plan & Execute按钮）
    add_rviz(ld, moveit_config)

    return ld

def add_move_group(ld, moveit_config):
    # --- 下面这些是launch里的可调参数 ---
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    # octomap 分辨率
    ld.add_action(
        DeclareLaunchArgument("octomap_resolution", default_value="0.005")
    )
    # 是否允许执行轨迹
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    # 是否往外广播规划场景（RViz那边的 MoveIt 面板会监听）
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # MoveGroup的插件能力开关（基本可以留空）
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    # 这个我们还是按False，不用它去盯动态学
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    # 把launch参数字符串转成float传给move_group
    octo_res = ParameterValue(
        LaunchConfiguration("octomap_resolution"),
        value_type=float,
    )
    # 这是 move_group 自己的一些行为配置
    move_group_configuration = {
        # 把 SRDF (semantic) 发布出去，RViz要用
        "publish_robot_description_semantic": True,

        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),

        # 下面两个需要 ParameterValue 包一下，否则空字符串传不了
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),

        # RViz/PlanningScene 同步需要的开关
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,

        "monitor_dynamics": False,
    }

    # MoveIt 执行轨迹时的行为
    trajectory_execution = {
        # 重要：我们不让 MoveIt 去“管理控制器生命周期”
        # 因为你不是 ros2_control 的 controller_manager，而是 rm_control / rm_driver
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    move_group_params = [
        moveit_config.to_dict(),           # robot_description, SRDF, kinematics, planning_pipelines...
        move_group_configuration,
        trajectory_execution,
        {"octomap_resolution": octo_res},
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # 这环境变量只是为了某些内部OpenGL调用不崩，一般保留也没坏处
        additional_env={"DISPLAY": ":0"},
    )


def add_rviz(ld, moveit_config):

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="screen",
        respawn=False,
        # 让RViz加载你配置助手导出的moveit.rviz布局
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )
