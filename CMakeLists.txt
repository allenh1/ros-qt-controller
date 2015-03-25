cmake_minimum_required(VERSION 2.8.9)
project(ros_qt_controller)
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Qt5Widgets)
find_package(Qt5Core)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS system-lib
)

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS src/*.h)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
	FOLLOW_SYMLINKS qt_resources/*.qrc)

qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    FOLLOW_SYMLINKS src/*.cpp)

include_directories(src ${catkin_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_executable(qt_ros_ctrl ${QT_SOURCES} ${QT_RESOURCES_CPP}
              ${QT_MOC_HPP})
target_link_libraries(qt_ros_ctrl ${QT_LIBRARIES}
            		  ${catkin_LIBRARIES} Qt5::Widgets)

install(TARGETS qt_ros_ctrl
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)