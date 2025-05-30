cmake_minimum_required(VERSION 3.8)
project(morve_hardware_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# lib hardware_error
add_library(hardware_error INTERFACE)
target_include_directories(hardware_error
  INTERFACE
    #"<$BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )

# lib Encoder
add_library(Encoder SHARED src/Encoder/Encoder.cpp)
target_include_directories(Encoder
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )
target_link_libraries(Encoder INTERFACE wiringPi hardware_error)

# lib SpecializedEncoder
add_library(SpecializedEncoder SHARED src/Encoder/SpecializedEncoder.cpp)

target_link_libraries(SpecializedEncoder PUBLIC Encoder)
target_compile_features(SpecializedEncoder PUBLIC cxx_std_20)

# lib motor_hat, tb6612fng
add_library(MotorHat_Adafruit SHARED)
add_library(tb6612fng SHARED src/MotorHat_Adafruit/tb6612fng.cpp src/MotorHat_Adafruit/pca9685.cpp)
target_include_directories(tb6612fng
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )
target_link_libraries(tb6612fng PUBLIC wiringPi)
target_compile_features(tb6612fng PUBLIC cxx_std_20)
target_sources(MotorHat_Adafruit PRIVATE src/MotorHat_Adafruit/motor_hat.cpp)
target_link_libraries(MotorHat_Adafruit PUBLIC tb6612fng)

# lib Battery_Voltage
add_library(Battery_Voltage SHARED src/Battery_Voltage/minimal_ina219.cpp)
target_include_directories(Battery_Voltage
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"  
    )
target_link_libraries(Battery_Voltage PUBLIC wiringPi)
target_compile_features(Battery_Voltage PUBLIC cxx_std_20)

# lib Sonar
add_library(Sonar SHARED src/Sonar/HC_SR04.cpp)
target_include_directories(Sonar
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )
target_link_libraries(Sonar PUBLIC wiringPi)

# executables
add_executable(test_encoder test/test_encoder.cpp)
target_link_libraries(test_encoder PUBLIC SpecializedEncoder)

add_executable(test_motor_hat test/test_motor_hat.cpp)
target_link_libraries(test_motor_hat PUBLIC MotorHat_Adafruit)

add_executable(test_battery_voltage test/test_minimal_ina219.cpp)
target_link_libraries(test_battery_voltage PUBLIC Battery_Voltage)

add_executable(test_sonar test/test_sonar_timeout.cpp)
target_link_libraries(test_sonar  PUBLIC Sonar)

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

install(TARGETS test_encoder test_motor_hat test_battery_voltage test_sonar
  DESTINATION lib/${PROJECT_NAME}
)

# library install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS Encoder SpecializedEncoder tb6612fng MotorHat_Adafruit Sonar Battery_Voltage hardware_error
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies()

ament_package()
