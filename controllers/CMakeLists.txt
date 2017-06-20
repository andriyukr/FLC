cmake_minimum_required(VERSION 2.8.3)
project(controllers)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
#file(GLOB_RECURSE EXTRA_FILES */*)
#add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs sensor_msgs geometry_msgs tf dynamic_reconfigure aruco_mapping)

generate_dynamic_reconfigure_options(cfg/setDIT1FLC.cfg cfg/setSIT2FLC.cfg cfg/setDIT2FLC.cfg cfg/setPID.cfg cfg/setTrajectory.cfg)

catkin_package()

# Depend on system install of Gazebo
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories(include "/opt/ros/kinetic/include" ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(si_it2_flc src/SI-IT2-FLC.cpp)
target_link_libraries(si_it2_flc ${catkin_LIBRARIES})
add_dependencies(si_it2_flc controllers_generate_messages_cpp)

add_executable(di_it2_flc src/DI-IT2-FLC.cpp)
target_link_libraries(di_it2_flc ${catkin_LIBRARIES})
add_dependencies(di_it2_flc controllers_generate_messages_cpp)

add_executable(di_it2_flc_fm src/DI-IT2-FLC-FM.cpp)
target_link_libraries(di_it2_flc_fm ${catkin_LIBRARIES})
add_dependencies(di_it2_flc_fm controllers_generate_messages_cpp)

add_executable(di_t1_flc_fm src/DI-T1-FLC-FM.cpp)
target_link_libraries(di_t1_flc_fm ${catkin_LIBRARIES})
add_dependencies(di_t1_flc_fm controllers_generate_messages_cpp)

add_executable(pid src/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})
add_dependencies(pid controllers_generate_messages_cpp)

add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})
add_dependencies(trajectory controllers_generate_messages_cpp)

add_executable(safe_bebop src/safeBebop.cpp)
target_link_libraries(safe_bebop ${catkin_LIBRARIES})
add_dependencies(safe_bebop controllers_generate_messages_cpp)

add_executable(teleop_key src/teleopKey.cpp)
target_link_libraries(teleop_key ${catkin_LIBRARIES})
add_dependencies(teleop_key controllers_generate_messages_cpp)
