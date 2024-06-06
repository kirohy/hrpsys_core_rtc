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

add_library(StateHolder SHARED
  StateHolder.cpp
  StateHolderService_impl.cpp
  TimeKeeperService_impl.cpp
)

# target_include_directories(StateHolder PUBLIC
#   ${PROJECT_SOURCE_DIR}/include
# )

target_link_libraries(StateHolder
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
)

set_target_properties(StateHolder PROPERTIES PREFIX "")
add_dependencies(StateHolder RTMBUILD2_${PROJECT_NAME}_genrpc)

add_executable(StateHolderComp StateHolderComp.cpp)
target_link_libraries(StateHolderComp StateHolder)

install(TARGETS StateHolder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

