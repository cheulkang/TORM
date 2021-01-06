cmake_minimum_required(VERSION 2.8.3)
project(torm)

add_definitions(-std=c++11 -O3)
#add_definitions(-std=c++11)

# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        trac_ik_lib
        eigen_conversions
        )

find_package(ompl REQUIRED)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} moveit_collision_distance_field
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/torm_cost.cpp
        src/torm_parameters.cpp
        src/torm_trajectory.cpp
        src/torm_optimizer.cpp
        src/torm_ik_solver.cpp
        src/torm_debug.cpp
        src/torm_problem.cpp
        )

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

add_executable(main src/torm_main.cpp)
target_link_libraries(main
        ${catkin_LIBRARIES} ${PROJECT_NAME}
        ${orocos_kdl_LIBRARIES}
        )