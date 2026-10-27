# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target actuator_msgs::actuator_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${actuator_msgs_TARGETS}.
if(actuator_msgs_TARGETS AND NOT TARGET actuator_msgs::actuator_msgs)
  add_library(actuator_msgs::actuator_msgs INTERFACE IMPORTED)
  set_target_properties(actuator_msgs::actuator_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${actuator_msgs_TARGETS}")
endif()
