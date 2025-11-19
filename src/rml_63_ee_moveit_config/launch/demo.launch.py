from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rml_63_ee_description", package_name="rml_63_ee_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
# generate_launch_description这个函数包含了move_group和rviz的启动
