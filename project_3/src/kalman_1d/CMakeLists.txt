cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(kalman_example VERSION 1.0)

# add the executable
add_executable(kalman_example main.cpp)

target_include_directories(kalman_example PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           )

# C++ Standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# executable

