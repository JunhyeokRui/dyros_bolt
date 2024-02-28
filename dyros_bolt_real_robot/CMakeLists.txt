cmake_minimum_required(VERSION 3.0.2)
project(dyros_bolt_real_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O)

# ADD_DEFINITIONS(-DTIME_CHECK)
# ADD_DEFINITIONS(-DECAT_DC)

execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RES OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- kernel version : " ${UNAME_RES})
#execute_process(COMMAND xeno-config --posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

#execute_process(COMMAND xeno-config --posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
#message(" xeno flags : " ${XENO_CFLAGS})
# string(FIND ${UNAME_RES} "xenomai" wsloc)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


# if(${wsloc} EQUAL -1 )
#   message(" simulation only compile mode ")
#   ADD_DEFINITIONS(-DCOMPILE_SIMULATION)
#   set(COMPILE_SIM TRUE)
# else()
#   message(" xenomai detected realrobot compile mode ")
#   ADD_DEFINITIONS(-DCOMPILE_REALROBOT)
#   set(COMPILE_REAL TRUE)
# endif() 

# find_package(Xenomai REQUIRED)
# find_package(catkin REQUIRED COMPONENTS roscpp)
# find_package(dyros_bolt_lib COMPONENTS
#     roscpp
#     dyros_bolt_msgs)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    dyros_bolt_msgs
    actionlib
)

# Assuming dyros_bolt_lib is a catkin package, not an external library
# If it's an external library, you should find it with find_package() and link it similarly
find_package(dyros_bolt_lib REQUIRED)

catkin_package(
   INCLUDE_DIRS include
)


catkin_package(
   INCLUDE_DIRS include 
)

# include_directories(
#   include
#   ${dyros_bolt_lib_INCLUDE_DIRS}
#   ${dyros_bolt_msgs_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}

  
# #   /usr/local/include/soem
# )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dyros_bolt_lib_INCLUDE_DIRS}
  ${dyros_bolt_msgs_INCLUDE_DIRS}
)

file(GLOB_RECURSE PROJECT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)


# file(GLOB BRIDGE_SRC
#     "src/real_robot_interface.cpp"
#     "src/control_base.cpp"
#     )
# file(GLOB CONTROLLER_SRC
#     "src/odrive_socketcan.cpp"
#     )

# add_library(dyros_bolt_real_robot_lib
#     ${BRIDGE_SRC}
#     ${CONTROLLER_SRC}
#     )

# # if(${COMPILE_REAL})

# link_directories(
# )
add_library(dyros_bolt_real_robot_lib ${PROJECT_SOURCES})

add_executable(dyros_bolt_real_robot src/main_real_robot.cpp)
# add_executable(tocabi_ecat_lower 
# src/master_lower.cpp src/ecat_master.cpp src/ecat_settings.cpp)
# add_executable(tocabi_ecat_upper 
# src/master_upper.cpp src/ecat_master.cpp src/ecat_settings.cpp)

# add_executable(red_test src/red_test.c)

# add_executable(watcher src/watcher.cpp)

# add_executable(watcher2 src/watcher_status.cpp)

# target_include_directories(dyros_bolt_real_robot PRIVATE ${XENOMAI_POSIX_INCLUDE_DIRS})
# target_include_directories(tocabi_ecat_lower PRIVATE ${XENOMAI_POSIX_INCLUDE_DIRS})
# target_include_directories(tocabi_ecat_upper PRIVATE ${XENOMAI_POSIX_INCLUDE_DIRS})


# target_link_libraries(dyros_bolt_real_robot
#   ${catkin_LIBRARIES}
#   ${BRIDGE_SRC}
#   ${CONTROLLER_SRC}

# # -lsoem
# #   ${XENOMAI_POSIX_LIBRARIES}
#   #${XENO_CFLAGS}
#   #${XENO_LDFLAGS}
# )

target_link_libraries(dyros_bolt_real_robot
  dyros_bolt_real_robot_lib
  ${catkin_LIBRARIES}
  ${dyros_bolt_lib_LIBRARIES}
)

add_dependencies(dyros_bolt_real_robot ${catkin_EXPORTED_TARGETS})

# target_link_libraries(tocabi_ecat_lower
#   -lsoem
#   ${XENOMAI_POSIX_LIBRARIES}

#   #${XENO_CFLAGS}
#   #${XENO_LDFLAGS}
# )
# target_link_libraries(tocabi_ecat_upper
#   -lsoem 
#   ${XENOMAI_POSIX_LIBRARIES}
#   #${XENO_CFLAGS}
#   #${XENO_LDFLAGS}
# )

# target_compile_definitions(dyros_bolt_real_robot PRIVATE ${XENOMAI_POSIX_DEFINITIONS})
# target_compile_definitions(tocabi_ecat_lower PRIVATE ${XENOMAI_POSIX_DEFINITIONS})
# target_compile_definitions(tocabi_ecat_upper PRIVATE ${XENOMAI_POSIX_DEFINITIONS})


# endif()

# if(${COMPILE_SIM})

# add_executable(watcher src/watcher.cpp)
# endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
