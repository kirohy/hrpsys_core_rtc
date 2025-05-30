add_definitions(-Wno-deprecated)
add_definitions(-DROBOT_IOB_VERSION=4)

find_package(catkin REQUIRED COMPONENTS
  jaxon_shm_controller
)
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
)

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  ${openrtm_aist_LIBRARY_DIRS}
)

add_library(RobotHardware2 SHARED
  robot.cpp
  RobotHardware2.cpp
  RobotHardware2Service_impl.cpp
)

target_link_libraries(RobotHardware2
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
)

set_target_properties(RobotHardware2 PROPERTIES PREFIX "")
add_dependencies(RobotHardware2 RTMBUILD2_${PROJECT_NAME}_genrpc)

add_executable(RobotHardware2Comp RobotHardware2Comp.cpp)
target_link_libraries(RobotHardware2Comp RobotHardware2)

install(TARGETS RobotHardware2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} CONFIGURATIONS Release Debug RelWithDebInfo
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION} CONFIGURATIONS Release Debug RelWithDebInfo
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

