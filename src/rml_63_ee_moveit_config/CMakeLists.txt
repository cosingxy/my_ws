cmake_minimum_required(VERSION 3.22)
project(rml_63_ee_moveit_config)

find_package(ament_cmake REQUIRED)
# 修改启动参数时需要用到的包
# find_package(launch REQUIRED)
# find_package(launch_ros REQUIRED)
# find_package(moveit_configs_utils REQUIRED)

ament_package()

#高级写法，把整个launch目录安装进去
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
