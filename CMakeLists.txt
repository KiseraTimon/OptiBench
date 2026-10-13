cmake_minimum_required(VERSION 3.20)
project(OptiBench CXX)

set(CMAKE_CXX_STANDARD 20)

# Core library containing shared headers and sources
add_library(optibench_lib
        src/Framebuffer.cpp
        include/optibench/Color.h
        include/optibench/Vec2.h
        include/optibench/Ray.h
        include/optibench/Framebuffer.h
        include/optibench/circle_ellipse.h
        src/circle_ellipse.cpp
)

# Exposed include directory
target_include_directories(optibench_lib PUBLIC include)

# Main Application Target
add_executable(OptiBench main.cpp)
target_link_libraries(OptiBench PRIVATE optibench_lib)

# Test Target
add_executable(optibench_tests
        tests/test_framebuffer.cpp
        tests/test_circle_ellipse.cpp
)
target_link_libraries(optibench_tests PRIVATE optibench_lib)

# Demo Target
add_executable(optibench_demos
        demos/ok_test.cpp
        demos/circle_ellipse.cpp
)
target_link_libraries(optibench_demos PRIVATE optibench_lib)