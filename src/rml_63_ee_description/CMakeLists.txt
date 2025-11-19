cmake_minimum_required(VERSION 3.10)
project(rml_63_ee_description)

find_package(ament_cmake REQUIRED)


# Install URDF and launch directories
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install description files
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
