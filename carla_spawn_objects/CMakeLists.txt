cmake_minimum_required(VERSION 3.5)
project(carla_spawn_objects)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

  catkin_python_setup()

  if(CATKIN_ENABLE_TESTING)
    roslaunch_add_file_check(launch)
  endif()

  catkin_package(CATKIN_DEPENDS rospy)

  catkin_install_python(
    PROGRAMS src/carla_spawn_objects/carla_spawn_objects.py
    src/carla_spawn_objects/set_initial_pose.py DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

  install(DIRECTORY config/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

elseif(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rclpy REQUIRED)
  ament_export_dependencies(rclpy)
  
  file(GLOB CONFIG_FILES "config/*.json")
  install(FILES ${CONFIG_FILES} DESTINATION share/${PROJECT_NAME}/config)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
  
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
        
  ament_package()
        
endif()
