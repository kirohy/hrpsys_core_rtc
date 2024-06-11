set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
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

add_library(StateHolder2 SHARED
  StateHolder2.cpp
  StateHolder2Service_impl.cpp
  TimeKeeper2Service_impl.cpp
)

target_link_libraries(StateHolder2
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
)

set_target_properties(StateHolder2 PROPERTIES PREFIX "")
add_dependencies(StateHolder2 RTMBUILD2_${PROJECT_NAME}_genrpc)

add_executable(StateHolder2Comp StateHolder2Comp.cpp)
target_link_libraries(StateHolder2Comp StateHolder2)

install(TARGETS StateHolder2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

