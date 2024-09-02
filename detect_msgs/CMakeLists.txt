cmake_minimum_required(VERSION 3.0.2)
project(detect_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

 add_message_files(
   DIRECTORY msg
   FILES
   detected_array.msg
   detected_object.msg
   Objects.msg
   Yolo_Objects.msg
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS 
  std_msgs
  geometry_msgs
  message_runtime
)
