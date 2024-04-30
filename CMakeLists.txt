cmake_minimum_required(VERSION 3.8)
project(secured_1d_velocity_controller LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    generate_parameter_library
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
    std_srvs
    secured_1d_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  secured_1d_velocity_controller_parameters
  src/secured_1d_velocity_controller.yaml
  include/secured_1d_velocity_controller/validate_secured_1d_velocity_controller_parameters.hpp
)

add_library(secured_1d_velocity_controller SHARED
            src/secured_1d_velocity_controller.cpp)

target_compile_features(secured_1d_velocity_controller PUBLIC cxx_std_17)
target_include_directories(
  secured_1d_velocity_controller
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(secured_1d_velocity_controller
                      PUBLIC secured_1d_velocity_controller_parameters)
ament_target_dependencies(secured_1d_velocity_controller PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_compile_definitions(
  secured_1d_velocity_controller
  PRIVATE "SECURED_1D_VELOCITY_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface
  secured_1d_velocity_controller.xml)

if(BUILD_TESTING)
  # Run all tests: colcon test --ctest-args tests --rerun-failed
  # --output-on-failure --packages-select secured_1d_velocity_controller View
  # test results: colcon test-result --verbose --test-result-base
  # build/secured_1d_velocity_controller

  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Test if the controller can be loaded
  set(test_load_sources test/test_load_secured_1d_velocity_controller.cpp)
  set(test_load_params
  ${CMAKE_CURRENT_SOURCE_DIR}/test/secured_1d_velocity_controller_params.yaml)

  add_rostest_with_parameters_gmock(test_load_secured_1d_velocity_controller
    ${test_load_sources}
    ${test_load_params}
  )

  ament_target_dependencies(test_load_secured_1d_velocity_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )


  # Test other aspects of the controller
  set(test_sources1 test/test_secured_1d_velocity_controller.cpp)
  set(test_params1
  ${CMAKE_CURRENT_SOURCE_DIR}/test/secured_1d_velocity_controller_params.yaml)

  add_rostest_with_parameters_gmock( test_secured_1d_velocity_controller
  ${test_sources1} ${test_params1} )

  target_include_directories(test_secured_1d_velocity_controller PUBLIC
  ${GMOCK_INCLUDE_DIRS})
  target_link_libraries(test_secured_1d_velocity_controller
  secured_1d_velocity_controller)

  ament_target_dependencies(test_secured_1d_velocity_controller
    controller_interface
    hardware_interface
    ros2_control_test_assets
  )

  # set(test_sources2  test/test_secured_1d_velocity_controller_preceeding.cpp)
  # set(test_params2
  # ${CMAKE_CURRENT_SOURCE_DIR}/test/secured_1d_velocity_controller_preceeding_params.yaml)

  # add_rostest_with_parameters_gmock(test_secured_1d_velocity_controller_preceeding
  # ${test_sources2} ${test_params2})

  # target_include_directories(test_secured_1d_velocity_controller_preceeding
  # PRIVATE include)
  # target_link_libraries(test_secured_1d_velocity_controller_preceeding
  # secured_1d_velocity_controller) ament_target_dependencies(
  # test_secured_1d_velocity_controller_preceeding controller_interface
  # hardware_interface )
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS secured_1d_velocity_controller
          secured_1d_velocity_controller_parameters
  EXPORT export_secured_1d_velocity_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_include_directories(include)

ament_export_targets(export_secured_1d_velocity_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
