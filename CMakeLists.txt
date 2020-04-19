cmake_minimum_required(VERSION 3.14)
project(bootload)

set(CMAKE_CXX_STANDARD 14)
add_link_options(-pthread)

include_directories(include)

add_executable(bootload include/bootloader.h src/bootloader.cpp src/hexreader.cpp include/hexreader.h src/queue.cpp)
