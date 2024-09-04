cmake_minimum_required(VERSION 3.16)
project(brakable_velocity_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  parameter_traits
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(brakable_velocity_controller_parameters
  src/brakable_velocity_controller.yaml
)

add_library(brakable_velocity_controller SHARED
  src/brakable_velocity_controller.cpp
)
target_compile_features(brakable_velocity_controller PUBLIC cxx_std_17)
target_include_directories(brakable_velocity_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/brakable_velocity_controller>
)
target_link_libraries(brakable_velocity_controller PUBLIC
  brakable_velocity_controller_parameters
)
ament_target_dependencies(brakable_velocity_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(brakable_velocity_controller PRIVATE "BRAKABLE_VELOCITY_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface brakable_velocity_controller.xml)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(controller_manager REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)

#   add_rostest_with_parameters_gmock(
#     test_brakable_velocity_controller
#     test/test_brakable_velocity_controller.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/test/brakable_velocity_controller_params.yaml
#   )
#   target_include_directories(test_brakable_velocity_controller PRIVATE include)
#   target_link_libraries(test_brakable_velocity_controller brakable_velocity_controller)
#   ament_target_dependencies(
#     test_brakable_velocity_controller
#     controller_interface
#     hardware_interface
#   )

#   add_rostest_with_parameters_gmock(
#     test_brakable_velocity_controller_preceding
#     test/test_brakable_velocity_controller_preceding.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/test/brakable_velocity_controller_preceding_params.yaml
#   )
#   target_include_directories(test_brakable_velocity_controller_preceding PRIVATE include)
#   target_link_libraries(test_brakable_velocity_controller_preceding brakable_velocity_controller)
#   ament_target_dependencies(
#     test_brakable_velocity_controller_preceding
#     controller_interface
#     hardware_interface
#   )

#   ament_add_gmock(test_load_brakable_velocity_controller test/test_load_brakable_velocity_controller.cpp)
#   target_include_directories(test_load_brakable_velocity_controller PRIVATE include)
#   ament_target_dependencies(
#     test_load_brakable_velocity_controller
#     controller_manager
#     ros2_control_test_assets
#   )
# endif()

install(
  DIRECTORY include/
  DESTINATION include/brakable_velocity_controller
)

install(TARGETS
    brakable_velocity_controller
    brakable_velocity_controller_parameters
  EXPORT export_brakable_velocity_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_brakable_velocity_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
