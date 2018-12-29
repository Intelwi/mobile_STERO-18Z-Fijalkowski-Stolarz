cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	std_msgs
	geometry_msgs
	message_generation
	tf
	costmap_2d
	move_base
)

add_message_files(
	#DIRECTORY msg
	FILES
	Whisper.msg
)

add_service_files(
	FILES
	CallItAsUWantIt.srv
	Positioning.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
	#INCLUDE_DIRS include
	#LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS message_runtime
	DEPENDS elektron
)

include_directories(include
	${catkin_INCLUDE_DIRS})

set(COSTMAP_NODE_SOURCES
	src/proj2/costmap.cpp
)

add_executable(costmap ${COSTMAP_NODE_SOURCES})
target_link_libraries(costmap ${catkin_LIBRARIES})

add_executable(footprint src/proj2/footprint.cpp)
target_link_libraries(footprint ${catkin_LIBRARIES})

add_executable(alles_maker src/proj2/alles_maker.cpp)
target_link_libraries(alles_maker ${catkin_LIBRARIES})

install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
