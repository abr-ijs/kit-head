cmake_minimum_required(VERSION 2.8.3)
project(kit_head_hw)

## DEPENDENCIES

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_toolbox
  controller_manager
  hardware_interface 
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
)

find_package(gazebo REQUIRED)

## CATKIN CONFIGURATION

catkin_package(
  CATKIN_DEPENDS
    roscpp
    controller_manager
    pluginlib
    transmission_interface
  INCLUDE_DIRS include
  LIBRARIES kit_head_hw default_kit_head_hw_sim
  DEPENDS gazebo
)

## BUILD

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## the base class
add_library(${PROJECT_NAME} src/kit_head_hw.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# the default sim behavior
add_library(default_kit_head_hw_sim src/default_kit_head_hw_sim.cpp)
target_link_libraries(default_kit_head_hw_sim 
	kit_head_hw 
	${catkin_LIBRARIES} 
	${GAZEBO_LIBRARIES}
)

# the plugin
add_library(kit_head_hw_sim_plugin src/kit_head_hw_sim_plugin.cpp)
target_link_libraries(kit_head_hw_sim_plugin
	kit_head_hw
	${catkin_LIBRARIES} 
	${GAZEBO_LIBRARIES}
)

## INSTALL

install(TARGETS ${PROJECT_NAME} default_kit_head_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES kit_head_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

