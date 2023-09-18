cmake_minimum_required(VERSION 3.0.2)
project(uav_msgs)

set(MSG_DEPS
  std_msgs
  actionlib_msgs
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
  actionlib_msgs
)

set(MSG_FILES
  BatteryStatus.msg
  InputAccepted.msg
)

add_service_files(
  FILES
  PrepareDrone.srv
)

add_action_files(
  DIRECTORY action
  FILES FlyToWP.action SpecialMovement.action
)

add_message_files(DIRECTORY msg FILES ${MSG_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})
