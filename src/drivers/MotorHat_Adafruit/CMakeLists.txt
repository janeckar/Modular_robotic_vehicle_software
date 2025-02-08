add_library(MotorHat_Adafruit STATIC)

add_library(tb6612fng STATIC tb6612fng.cpp pca9685.cpp)

# state that anybody that is linking to this needs to include current dirrectory
target_include_directories(tb6612fng INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(MotorHat_Adafruit PRIVATE motor_hat.cpp)

target_link_libraries(MotorHat_Adafruit PUBLIC tb6612fng)