cmake_minimum_required(VERSION 2.8.3)
project(ros_qt_controller)
find_package(catkin REQUIRED COMPONENTS qt_build roscpp)

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp geometry_msgs tf
  DEPENDS system-lib
)
rosbuild_prepare_qt4(QtCore QtGui)

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS src/*.h)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
	FOLLOW_SYMLINKS qt_resources/*.qrc)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    FOLLOW_SYMLINKS src/*.cpp)

include_directories(src ${catkin_INCLUDE_DIRS})

add_executable(qt_ros_ctrl ${QT_SOURCES} ${QT_RESOURCES_CPP}
              ${QT_MOC_HPP})
target_link_libraries(qt_ros_ctrl ${QT_LIBRARIES}
            		  ${catkin_LIBRARIES})

install(TARGETS qt_ros_ctrl
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)