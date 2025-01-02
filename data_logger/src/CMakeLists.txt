set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  robot_hardware
)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
)

link_directories(
  ${openrtm_aist_LIBRARY_DIRS}
)

add_library(DataLogger2 SHARED
  DataLogger2.cpp
  DataLogger2Service_impl.cpp
)

target_link_libraries(DataLogger2
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
)

set_target_properties(DataLogger2 PROPERTIES PREFIX "")
add_dependencies(DataLogger2 RTMBUILD2_${PROJECT_NAME}_genrpc)

add_executable(DataLogger2Comp DataLogger2Comp.cpp)
target_link_libraries(DataLogger2Comp DataLogger2)

install(TARGETS DataLogger2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} CONFIGURATIONS Release Debug RelWithDebInfo
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION} CONFIGURATIONS Release Debug RelWithDebInfo
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

