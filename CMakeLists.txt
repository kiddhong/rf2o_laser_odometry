cmake_minimum_required(VERSION 3.5)
PROJECT(rf2o_laser_odometry)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

# if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
# 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
# else()
# 	set(CMAKE_CXX_STANDARD 17)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

set(dependencies
rclcpp
std_msgs
sensor_msgs
geometry_msgs
nav_msgs
tf2
tf2_ros
tf2_msgs
tf2_geometry_msgs
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
src/CLaserOdometry2D.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node 
src/CLaserOdometry2DNode.cpp
)

target_link_libraries(${PROJECT_NAME}_node
${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node 
  ${dependencies}
)

# install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY include/
#   DESTINATION include/
# )

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# ament_export_include_directories(include)
# ament_export_libraries()
# ament_export_dependencies(${dependencies})

ament_package()
