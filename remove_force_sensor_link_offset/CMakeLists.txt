cmake_minimum_required(VERSION 3.0.2)
project(remove_force_sensor_link_offset)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp
)

# catkin_python_setup()
rtmbuild2_init(rtmbuild2)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES RemoveForceSensorLinkOffset2ServiceSkel RemoveForceSensorLinkOffset2ServiceStub
  CATKIN_DEPENDS rtmbuild2
  DEPENDS openrtm_aist
)

# generate idl
rtmbuild2_genidl()

# generate bridge
rtmbuild2_genbridge()

add_subdirectory(src)

