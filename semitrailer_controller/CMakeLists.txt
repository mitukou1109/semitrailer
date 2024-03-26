cmake_minimum_required(VERSION 3.8)
project(semitrailer_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

find_package(vehicle_model REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(semitrailer_nlmpc SHARED
  src/semitrailer_nlmpc.cpp
)
target_link_libraries(semitrailer_nlmpc
  vehicle_model::semitrailer_model
)

ament_auto_package()
