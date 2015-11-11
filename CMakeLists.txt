cmake_minimum_required(VERSION 2.8.3)
project(GuidanceRos)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
  geometry_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

MESSAGE("System is: " ${CMAKE_SYSTEM_PROCESSOR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if (CMAKE_SIZEOF_VOID_P EQUAL 4)
		MESSAGE("-- 32bit detected")	
		link_directories(${PROJECT_SOURCE_DIR}/lib/x86)
	elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
		MESSAGE("-- 64bit detected")
		link_directories(${PROJECT_SOURCE_DIR}/lib/x64)
	endif ()
else()
	MESSAGE("-- Non-linux platform detected but sorry we do not support :D")
endif ()

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l" )
	MESSAGE("-- " ${CMAKE_SYSTEM_PROCESSOR} " detected")
	link_directories(${PROJECT_SOURCE_DIR}/lib/XU3)
else ()
endif ()

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(GuidanceNode ${PROJECT_SOURCE_DIR}/src/GuidanceNode.cpp ${PROJECT_SOURCE_DIR}/src/DJI_utility.cpp)
add_executable(GuidanceNodeTest ${PROJECT_SOURCE_DIR}/src/GuidanceNodeTest.cpp)
add_executable(GuidanceNodeCalibration ${PROJECT_SOURCE_DIR}/src/GuidanceNodeCalibration.cpp ${PROJECT_SOURCE_DIR}/src/DJI_utility.cpp)

target_link_libraries(GuidanceNode DJI_guidance usb-1.0 ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(GuidanceNodeTest ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(GuidanceNodeCalibration DJI_guidance usb-1.0 yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBS})
