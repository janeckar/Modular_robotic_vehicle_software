add_library(Encoder STATIC Encoder.cpp)
target_include_directories(Encoder INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(SpecializedEncoder STATIC SpecializedEncoder.cpp)

target_link_libraries(SpecializedEncoder PUBLIC Encoder)