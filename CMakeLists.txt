cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(MY_GRAND_PROJECT)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(./src)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable(ICP_main src/PointCloudRelation.cpp src/PointCloudRelation.h src/PointCloudNode.cpp src/PointCloudNode.h src/ICPOperation.cpp src/ICPOperation.h src/ICPOptions.cpp src/ICPOptions.h src/ICPDefine.h)
#add_executable(ICP_trans src/ICP_XYZ_Transformer.cpp)
add_executable(ICP_msg_handler src/ROS_msg_handler.cpp src/ROSmsg.cpp src/ROSmsg.h)

#target_link_libraries(ICP_main ${PCL_LIBRARIES})
#target_link_libraries(ICP_trans ${PCL_LIBRARIES})
target_link_libraries(ICP_msg_handler ${PCL_LIBRARIES})
