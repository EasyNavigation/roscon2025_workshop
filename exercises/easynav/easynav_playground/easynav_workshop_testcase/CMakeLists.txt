cmake_minimum_required(VERSION 3.20)
project(easynav_workshop_testcase)

find_package(ament_cmake REQUIRED)

install(DIRECTORY maps/ rviz/ config/ DESTINATION share/${PROJECT_NAME})

ament_package()
