cmake_minimum_required(VERSION 2.8)
project(python_h264decoder)

set(Python_ADDITIONAL_VERSIONS 2.7)
find_package(PythonInterp 2.7 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED )
find_package(Boost REQUIRED COMPONENTS "python")

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_compile_options ("-std=c++0x")

add_executable(h264decoder_test1 h264decoder.cpp h264decoder_test1.cpp)
target_link_libraries(h264decoder_test1 avcodec swscale avutil)

add_library(h264decoder SHARED h264decoder.cpp h264decoder_python.cpp)
target_link_libraries(h264decoder avcodec swscale avutil ${Boost_PYTHON_LIBRARY_RELEASE} ${PYTHON_LIBRARIES})


add_custom_command(TARGET h264decoder POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E create_symlink 
                   ${CMAKE_BINARY_DIR}/libh264decoder.so ${CMAKE_SOURCE_DIR}/libh264decoder.so)
install(TARGETS h264decoder LIBRARY DESTINATION .)
