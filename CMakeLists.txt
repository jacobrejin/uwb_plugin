cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# Set the custom installation path
# set(CMAKE_PREFIX_PATH "/custom/install/path" CACHE PATH "Custom install path")


find_package(ignition-cmake2 REQUIRED)

project(uwb_plugin_test_1)

ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})



# Find the custom Ignition Rendering library
find_package(ignition-rendering6 6.6.3 REQUIRED)
# Print the paths of the found package
if (ignition-rendering6_FOUND)
    message(STATUS "Found Ignition Rendering library:")
    message(STATUS "  Include dir: ${ignition-rendering6_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${ignition-rendering6_LIBRARIES}")
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)


# Include directories for ROS 2
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${ros_gz_INCLUDE_DIRS}
)



add_library(uwb_plugin SHARED UwbPlugin.cpp)
set_property(TARGET uwb_plugin PROPERTY CXX_STANDARD 17)



target_link_libraries(uwb_plugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PRIVATE ${rclcpp_LIBRARIES}
  PRIVATE ${visualization_msgs_LIBRARIES}
  )


ament_export_libraries(uwb_plugin_test_1)
ament_package()