cmake_minimum_required(VERSION 3.0.2)
project(uav_msgs)

set(MSG_DEPS
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
)

set(ACTION_FILES
  LeedsPumpGimbal.action
  LeedsPumpPump.action
  VisualControlTask.action
)

set(MSG_FILES
  ClosestObject.msg
  WallExtremities.msg
  LeedsPumpLevel.msg
  VisualControlProgress.msg
  BatteryStatus.msg
  InputAccepted.msg
)

set(SRV_FILES
  Report.srv
  CommandMisc.srv
  ExecuteWaypoint.srv
  SetLocalTarget.srv
)

add_action_files(DIRECTORY action FILES ${ACTION_FILES})
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})
