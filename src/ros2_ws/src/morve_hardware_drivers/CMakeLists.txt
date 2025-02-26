cmake_minimum_required(VERSION 3.8)
project(morve_hardware_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# lib Encoder
add_library(Encoder src/Encoder/Encoder.cpp)
target_include_directories(Encoder
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )
target_link_libraries(Encoder INTERFACE wiringPi)

# lib SpecializedEncoder
add_library(SpecializedEncoder src/Encoder/SpecializedEncoder.cpp)
target_link_libraries(SpecializedEncoder PUBLIC Encoder)
target_compile_features(SpecializedEncoder PUBLIC cxx_std_20)

# lib motor_hat, tb6612fng
add_library(MotorHat_Adafruit)
add_library(tb6612fng src/MotorHat_Adafruit/tb6612fng.cpp src/MotorHat_Adafruit/pca9685.cpp)
target_include_directories(tb6612fng
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )
target_sources(MotorHat_Adafruit PRIVATE src/MotorHat_Adafruit/motor_hat.cpp)
target_link_libraries(MotorHat_Adafruit PUBLIC tb6612fng)

# lib Sonar
add_library(Sonar src/Sonar/HC_SR04.cpp)
target_include_directories(Sonar
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )


# executables
add_executable(test_encoder test/test_encoder.cpp)
target_link_libraries(test_encoder PUBLIC SpecializedEncoder)


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
set(CMAKE_INSTALL_GROUP "gpio")

install(TARGETS test_encoder
  DESTINATION lib/${PROJECT_NAME}
)

# library install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS Encoder SpecializedEncoder tb6612fng MotorHat_Adafruit Sonar
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(wiringPi)

ament_package()
