cmake_minimum_required(VERSION 3.16)
project(murin_base LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(GCC_COVERAGE_LINK_FLAGS "-ljsoncpp")

# # COMPILE
add_library(
  murin_base
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(murin_base PUBLIC cxx_std_17)
target_include_directories(murin_base PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/murin_base>
)
ament_target_dependencies(
  murin_base PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(murin_base PUBLIC serial)
target_link_libraries(murin_base PUBLIC jsoncpp)
target_link_options(murin_base PUBLIC -ljsoncpp)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "MURIN_BASE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface murin_base.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/murin_base
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/murin_base
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/murin_base
)
install(TARGETS murin_base
  EXPORT export_murin_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# # EXPORTS
ament_export_targets(export_murin_base HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
