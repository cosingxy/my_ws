# Return a list of nodes we commonly launch for the demo. Nice for testing use.
import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

# 自己定义了一个方法，从指定包中加载 yaml 文件
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

# 自己定义了一个函数，从指定包中获取机器人描述（URDF）
# 包路径为 rml_63_ee_moveit_config/config/rml_63_ee_description.urdf.xacro
def get_robot_description():
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("rml_63_ee_moveit_config"),
            "config",
            "rml_63_ee_description.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    return robot_description

# 自己定义了一个函数，从指定包中获取机器人语义描述（SRDF）
# 包路径为 rml_63_ee_moveit_config/config/rml_63_ee_description.srdf   
def get_robot_description_semantic():
    robot_description_semantic_config = load_file(
        "rml_63_ee_moveit_config", "config/rml_63_ee_description.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    return robot_description_semantic


def generate_common_hybrid_launch_description():
    robot_description = get_robot_description()

    robot_description_semantic = get_robot_description_semantic()

    kinematics_yaml = load_yaml(
        "rml_63_ee_moveit_config", "config/kinematics.yaml"
    )

    # The global planner uses the typical OMPL parameters
    # 所有组件共享的参数
    # planning_group_name: "arm"  # 规划组名称
    # reference_frame: "base_link"  # 参考坐标系
    # controller_name: "arm_controller"  # 控制器名称
    planning_pipelines_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "rml_63_ee_moveit_config", "config/ompl_planning.yaml"
    )
    # 把 ompl_planning_yaml 的内容添加到 planning_pipelines_config["ompl"] 中
    planning_pipelines_config["ompl"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        "rml_63_ee_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Any parameters that are unique to your plugins go here
    common_hybrid_planning_param = load_yaml(
        "rm63_ee_hybrid_planning", "config/common_hybrid_planning_params.yaml"
    )
    global_planner_param = load_yaml(
        "rm63_ee_hybrid_planning", "config/global_planner.yaml"
    )
    local_planner_param = load_yaml(
        "rm63_ee_hybrid_planning", "config/local_planner.yaml"
    )
    hybrid_planning_manager_param = load_yaml(
        "rm63_ee_hybrid_planning", "config/hybrid_planning_manager.yaml"
    )

    # Generate launch description with multiple components
    container = ComposableNodeContainer(
        name="hybrid_planning_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[
                    common_hybrid_planning_param,
                    global_planner_param,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    planning_pipelines_config,
                    moveit_controllers,
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[
                    common_hybrid_planning_param,
                    local_planner_param,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[
                    common_hybrid_planning_param,
                    hybrid_planning_manager_param,
                ],
            ),
        ],
        output="screen",
    )

    # RViz - 启动空白配置，可以手动添加显示项
    # 如果想使用预设配置，取消下面两行的注释并注释掉 arguments=[] 这行
    rviz_config_file = (
        get_package_share_directory("rm63_ee_hybrid_planning")
        + "/config/hybrid_planning_demo.rviz.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=[],  # 空白启动，不加载配置文件
        arguments=["-d", rviz_config_file],  # 如需加载配置文件，注释上一行并取消此行注释
        parameters=[robot_description, robot_description_semantic],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
    get_package_share_directory("rml_63_ee_moveit_config"),
    "config",
    "ros2_controllers.yaml",  # 建议用你自己的文件名
)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "-c",
            "/controller_manager",
        ],
    )

    launched_nodes = [
        container,
        static_tf,
        rviz_node,
        robot_state_publisher,
        ros2_control_node,        # ☆ 重新启用
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]
    print("common_hybrid_planning_param is None?", common_hybrid_planning_param is None)
    print("global_planner_param is None?", global_planner_param is None)
    print("local_planner_param is None?", local_planner_param is None)
    print("hybrid_planning_manager_param is None?", hybrid_planning_manager_param is None)
    print("kinematics_yaml is None?", kinematics_yaml is None)
    print("moveit_simple_controllers_yaml is None?", moveit_simple_controllers_yaml is None)


    return launched_nodes
