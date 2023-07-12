cmake_minimum_required(VERSION 3.5)
project(multirobots_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)

nav2_package()


ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  multirobots_navigation/remapper.py
  multirobots_navigation/initialize_pose.py
  multirobots_navigation/navigate_to_random_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclpy)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
