find_package(catkin REQUIRED COMPONENTS
  cpp_filters
)
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

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

add_library(SequencePlayer SHARED
  SequencePlayer.cpp
  SequencePlayerService_impl.cpp
  interpolator.cpp
  seqplay.cpp
  timeUtil.cpp
)

target_link_libraries(SequencePlayer
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
)

set_target_properties(SequencePlayer PROPERTIES PREFIX "")
add_dependencies(SequencePlayer RTMBUILD2_${PROJECT_NAME}_genrpc)

add_executable(SequencePlayerComp SequencePlayerComp.cpp)
target_link_libraries(SequencePlayerComp SequencePlayer)

install(TARGETS SequencePlayer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

