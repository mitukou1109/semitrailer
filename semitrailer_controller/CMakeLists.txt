cmake_minimum_required(VERSION 3.8)
project(semitrailer_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

find_package(Ceres REQUIRED)

find_package(vehicle_model REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(semitrailer_nlmpc SHARED
  src/semitrailer_nlmpc.cpp
)
target_link_libraries(semitrailer_nlmpc
  Ceres::ceres
  vehicle_model::semitrailer_model
)
target_compile_features(semitrailer_nlmpc PRIVATE cxx_std_17)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/simple_semitrailer_navigation.cpp
)
target_link_libraries(${PROJECT_NAME}
  semitrailer_nlmpc
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "semitrailer_controller::SimpleSemitrailerNavigation"
  EXECUTABLE simple_semitrailer_navigation_node
)

ament_auto_package()
