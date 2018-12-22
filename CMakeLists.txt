cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	message_generation
)

add_message_files(
	#DIRECTORY msg
	FILES
	Whisper.msg
)

add_service_files(
	FILES
	CallItAsUWantIt.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	#INCLUDE_DIRS include
	#LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS message_runtime
	DEPENDS elektron
)

install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
