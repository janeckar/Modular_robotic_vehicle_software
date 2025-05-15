cmake_minimum_required(VERSION 3.8)
project(sonar_distance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  generate_parameter_library
  parameter_traits
  rclcpp
  sensor_msgs
  morve_hardware_drivers
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(sonar_node_parameters
  src/sonar_node_parameters.yaml
)

add_executable(sonar_publisher
  src/sonar_node.cpp
)
target_compile_features(sonar_publisher PUBLIC cxx_std_17)
target_include_directories(sonar_publisher PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(sonar_publisher PUBLIC
  sonar_node_parameters
)
ament_target_dependencies(sonar_publisher PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

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

# executable install
install(TARGETS sonar_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# library install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS sonar_node_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
