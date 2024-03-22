cmake_minimum_required(VERSION 3.8)
project(semitrailer_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

find_package(vehicle_model REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/semitrailer_simulator.cpp
)
target_link_libraries(${PROJECT_NAME}
  vehicle_model::semitrailer_model
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "semitrailer_simulation::SemitrailerSimulator"
  EXECUTABLE semitrailer_simulator_node
)

ament_auto_package(
  INSTALL_TO_SHARE config launch rviz
)
