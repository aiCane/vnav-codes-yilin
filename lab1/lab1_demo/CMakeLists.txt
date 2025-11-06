cmake_minimum_required(VERSION 3.10)

project(
  hello_world
  VERSION 1.0
  LANGUAGES CXX)

add_library(mylib my_lib.cpp my_lib.hpp)
add_executable(main main.cpp)
target_link_libraries(main PRIVATE mylib)
