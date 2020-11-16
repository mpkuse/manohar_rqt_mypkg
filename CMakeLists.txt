cmake_minimum_required(VERSION 3.0.2)
project(manohar_rqt_mypkg)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES manohar_rqt_mypkg
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(  ${catkin_INCLUDE_DIRS} )


#############
## Install ##
#############
install(PROGRAMS scripts/manohar_rqt_mypkg
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
