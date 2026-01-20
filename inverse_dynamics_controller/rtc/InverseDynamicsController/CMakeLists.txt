set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  cpp_filters
  hrpsys_ros_bridge
  )
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  )
link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_library(InverseDynamicsController SHARED
  InverseDynamicsController.cpp
  InverseDynamicsControllerService_impl.cpp
  )
target_link_libraries(InverseDynamicsController
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
  # RobotHardwareServiceSkel RobotHardwareServiceStub
  )
set_target_properties(InverseDynamicsController PROPERTIES PREFIX "")
add_dependencies(InverseDynamicsController RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild2_genidl

# add_executable(InverseDynamicsControllerComp InverseDynamicsControllerComp.cpp)
# target_link_libraries(InverseDynamicsControllerComp InverseDynamicsController)

install(TARGETS InverseDynamicsController
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
