cmake_minimum_required(VERSION 3.8)
project(skyscan_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_sim.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
