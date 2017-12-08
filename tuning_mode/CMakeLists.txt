cmake_minimum_required(VERSION 2.8.3)
project(rosplane_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  rosplane_msgs
    rospy
    rqt_gui
    rqt_gui_py
)
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES tuning_mode
        #  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
        #  DEPENDS system_lib
)
catkin_python_setup()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS src/tuning_mode.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
#install(DIRECTORY launch resource
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )
install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
