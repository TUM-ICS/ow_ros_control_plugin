cmake_minimum_required(VERSION 3.11)
project(ow_ros_control_plugin)

###################
## Configuration ##
###################

# ROS dependecies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    ow_core
    ow_hw_interface
    ow_controller
    hardware_interface
    controller_interface
)

# System dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# Package configuration (this information is passed to projects/packages that
# depend on this package)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ow_ros_control_plugin
    CATKIN_DEPENDS
        roscpp
        ow_core
        ow_hw_interface
        ow_controller
        hardware_interface
        controller_interface
#    DEPENDS ${DEPENDS_VAR}
)

###########
## Build ##
###########

# Location of header files (First package headers always!)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)



# add executables
set(SOURCES
    src/ros_control_plugin.cpp
)

add_library(${PROJECT_NAME}  ${SOURCES})

# Library dependencies. Always use:
# ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

# Special CXXFlags for the libraries
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11 )

# linking libraries (from ros packages and system libraries)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#set(APP_TARGETS
#    ${PROJECT_NAME}_test1
#)

#foreach(target ${APP_TARGETS})
#        add_executable(${target} src/applications/main_${target}.cpp)
#        add_dependencies(${target}
#            ${${PROJECT_NAME}_EXPORTED_TARGETS}
#            ${catkin_EXPORTED_TARGETS})

#        target_link_libraries(${target}
#            ${PROJECT_NAME}
#            ${catkin_LIBRARIES}
#        )
#endforeach()

#############
## Install ##
#############

# don't install private headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN "*_p.h" EXCLUDE
    PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} ${APP_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )



install(FILES ow_ros_control_plugin_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
set(TEST_TARGETS
  #ros_control_plugin
)

foreach(target ${TEST_TARGETS})
  catkin_add_gtest(${target} src/tests/test_${target}.cpp)
  add_dependencies(${target}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS})

  set_target_properties(${target} PROPERTIES CXX_STANDARD 11)

  target_link_libraries(${target}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
  )
endforeach()

endif()

