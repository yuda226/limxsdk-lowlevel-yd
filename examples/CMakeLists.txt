cmake_minimum_required(VERSION 3.5)

set(EXAMPLES_BIN_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/bin)
set(EXAMPLES_LIB_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/lib)

if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_VERSION} EQUAL 1)
    set(EXAMPLES_BIN_INSTALL_PREFIX ${CATKIN_PACKAGE_BIN_DESTINATION})
    set(EXAMPLES_LIB_INSTALL_PREFIX ${CATKIN_GLOBAL_LIB_DESTINATION})
  elseif($ENV{ROS_VERSION} EQUAL 2)
    set(EXAMPLES_BIN_INSTALL_PREFIX lib/${PROJECT_NAME})
    set(EXAMPLES_LIB_INSTALL_PREFIX lib)
  endif()
endif()

find_package(Eigen3 REQUIRED)

include_directories(./)
include_directories(${EIGEN3_INCLUDE_DIR})

set(COMMON_SRCS
  ./pf_controller_base.cpp
)

if (WIN32)
  set(LINK_LIBS limxsdk_lowlevel shlwapi)
else()
  set(LINK_LIBS limxsdk_lowlevel pthread)
endif()

add_executable(pf_joint_move pf_joint_move.cpp ${COMMON_SRCS})
target_link_libraries(pf_joint_move ${LINK_LIBS})
install(TARGETS pf_joint_move DESTINATION ${EXAMPLES_BIN_INSTALL_PREFIX})

add_executable(pf_groupJoints_move pf_groupJoints_move.cpp ${COMMON_SRCS})
target_link_libraries(pf_groupJoints_move ${LINK_LIBS})
install(TARGETS pf_groupJoints_move DESTINATION ${EXAMPLES_BIN_INSTALL_PREFIX})

if(DEFINED ENV{ROS_DISTRO} AND DEFINED ENV{ROS_VERSION} AND $ENV{ROS_VERSION} EQUAL 2)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(nlohmann_json REQUIRED)

  add_executable(tron2_cmd_vel_translator tron2_cmd_vel_translator.cpp)
  target_link_libraries(tron2_cmd_vel_translator ${LINK_LIBS})
  ament_target_dependencies(tron2_cmd_vel_translator rclcpp geometry_msgs)
  install(TARGETS tron2_cmd_vel_translator DESTINATION ${EXAMPLES_BIN_INSTALL_PREFIX})

  add_executable(tron2_request_mode tron2_request_mode.cpp)
  target_link_libraries(tron2_request_mode ${LINK_LIBS})
  ament_target_dependencies(tron2_request_mode rclcpp geometry_msgs std_msgs std_srvs nlohmann_json)
  install(TARGETS tron2_request_mode DESTINATION ${EXAMPLES_BIN_INSTALL_PREFIX})

  add_executable(tron2_control_mode tron2_control_mode.cpp)
  target_compile_features(tron2_control_mode PRIVATE cxx_std_17)
  target_link_libraries(tron2_control_mode ${LINK_LIBS})
  ament_target_dependencies(tron2_control_mode rclcpp geometry_msgs std_msgs std_srvs nlohmann_json)
  install(TARGETS tron2_control_mode DESTINATION ${EXAMPLES_BIN_INSTALL_PREFIX})

  if(BUILD_TESTING)
    enable_testing()
    add_executable(test_tron2_control_mode test_tron2_control_mode.cpp)
    target_compile_features(test_tron2_control_mode PRIVATE cxx_std_17)
    target_link_libraries(test_tron2_control_mode ${LINK_LIBS})
    ament_target_dependencies(test_tron2_control_mode rclcpp geometry_msgs std_msgs std_srvs nlohmann_json)
    add_test(NAME tron2_control_mode COMMAND test_tron2_control_mode)
    set_tests_properties(tron2_control_mode PROPERTIES TIMEOUT 20)
  endif()
endif()

