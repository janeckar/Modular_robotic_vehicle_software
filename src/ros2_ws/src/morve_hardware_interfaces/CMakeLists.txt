cmake_minimum_required(VERSION 3.8)
project(morve_hardware_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(morve_hardware_drivers REQUIRED)

# add shared library
add_library(morve_hardware_interfaces SHARED src/motor_interface.cpp)
target_include_directories(morve_hardware_interfaces
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )
target_compile_features(morve_hardware_interfaces PUBLIC cxx_std_11)
ament_target_dependencies(morve_hardware_interfaces
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle 
  morve_hardware_drivers
)
pluginlib_export_plugin_description_file(hardware_interface
  morve_hardware_interfaces.xml)


add_executable(test_stoi_hex src/test_stoi_hex.cpp)
target_compile_features(test_stoi_hex PUBLIC cxx_std_20)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# executables install
install(TARGETS test_stoi_hex
  DESTINATION lib/${PROJECT_NAME}
)

# library install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

install(TARGETS morve_hardware_interfaces
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(morve_hardware_drivers)

ament_package()
