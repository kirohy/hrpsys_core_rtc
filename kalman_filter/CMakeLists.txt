cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp
)

rtmbuild2_init(rtmbuild2)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES KalmanFilter2ServiceSkel KalmanFilter2ServiceStub
  CATKIN_DEPENDS rtmbuild2
  DEPENDS openrtm_aist
)

rtmbuild2_genidl()

rtmbuild2_genbridge()

add_subdirectory(src)

