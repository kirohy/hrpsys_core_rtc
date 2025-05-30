add_definitions(-Wno-deprecated)

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

add_library(KalmanFilter2 SHARED
  KalmanFilter2.cpp
  KalmanFilter2Service_impl.cpp
)

target_link_libraries(KalmanFilter2
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
)

set_target_properties(KalmanFilter2 PROPERTIES PREFIX "")
add_dependencies(KalmanFilter2 RTMBUILD2_${PROJECT_NAME}_genrpc)

add_executable(KalmanFilter2Comp KalmanFilter2Comp.cpp)
target_link_libraries(KalmanFilter2Comp KalmanFilter2)

install(TARGETS KalmanFilter2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

