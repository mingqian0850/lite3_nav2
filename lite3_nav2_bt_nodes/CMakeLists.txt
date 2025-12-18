cmake_minimum_required(VERSION 3.8)
project(lite3_nav2_bt_nodes)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/path_from_topic.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC BT_PLUGIN_EXPORT
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
  pluginlib
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(behaviortree_cpp_v3 bt_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  bt_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

