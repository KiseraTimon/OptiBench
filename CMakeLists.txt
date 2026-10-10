cmake_minimum_required(VERSION 3.20)
project(OptiBench CXX)

set(CMAKE_CXX_STANDARD 20)

# Core library containing shared headers and sources
add_library(optibench_lib
        src/Framebuffer.cpp
        src/Line.cpp
        include/optibench/Color.h
        include/optibench/Vec2.h
        include/optibench/Ray.h
        include/optibench/Framebuffer.h
        include/optibench/Line.h
)

# Exposed include directory
target_include_directories(optibench_lib PUBLIC include)

# Main Application Target
add_executable(OptiBench main.cpp)
target_link_libraries(OptiBench PRIVATE optibench_lib)

# Test Target
add_executable(test_framebuffer tests/test_framebuffer.cpp)
target_link_libraries(test_framebuffer PRIVATE optibench_lib)

# Demo Target
add_executable(ok_test demos/ok_test.cpp)
target_link_libraries(ok_test PRIVATE optibench_lib)

# Line Test Target
add_executable(test_line tests/test_line.cpp)
target_link_libraries(test_line PRIVATE optibench_lib)

# Line Demo Target
add_executable(line_demo demos/line_demo.cpp)
target_link_libraries(line_demo PRIVATE optibench_lib)
