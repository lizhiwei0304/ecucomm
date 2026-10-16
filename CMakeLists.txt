cmake_minimum_required(VERSION 2.8.3)
project(ecucomm)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++11 ${CMAKE_CXX_FLAGS}")
set(catkin_deps
        roscpp
		rospy
        geometry_msgs
        sensor_msgs
    )

find_package(catkin REQUIRED COMPONENTS
	${catkin_deps}
        message_generation
        std_msgs
        serial
	)

catkin_package(
	INCLUDE_DIRS 
	CATKIN_DEPENDS
	${catkin_deps}  
        message_runtime
        serial
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ecucomm.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )



