cmake_minimum_required(VERSION 3.20)
project(OptiBench CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Core library containing shared headers and sources
add_library(optibench_lib
        src/Framebuffer.cpp
        src/Polygon.cpp
        src/Line.cpp
        include/optibench/Color.h
        include/optibench/Vec2.h
        include/optibench/Ray.h
        include/optibench/Framebuffer.h
        include/optibench/Line.h
        include/optibench/circle_ellipse.h
        include/optibench/Polygon.h
        src/circle_ellipse.cpp
        include/optibench/clip_fill.h
        src/clip_fill.cpp
)
target_include_directories(optibench_lib PUBLIC include) # exposed libraries

# ---

# Main Application Target
add_executable(OptiBench main.cpp)
target_link_libraries(OptiBench PRIVATE optibench_lib)

# ---

# Test Targets
# 1. Framebuffer test executable
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
# 2. Circles/Ellipses test executable
add_executable(test_circle_ellipse tests/test_circle_ellipse.cpp)
target_link_libraries(test_circle_ellipse PRIVATE optibench_lib)

# 3. Polygon clipping/filling test executable
add_executable(test_clip_fill tests/test_clip_fill.cpp)
target_link_libraries(test_clip_fill PRIVATE optibench_lib)

# ---

# Demo Targets
# 1. ok test demo executable
add_executable(demo_ok_test demos/ok_test.cpp)
target_link_libraries(demo_ok_test PRIVATE optibench_lib)

# 2. circle/ellipse demo executable
add_executable(demo_circle_ellipse demos/circle_ellipse.cpp)
target_link_libraries(demo_circle_ellipse PRIVATE optibench_lib)



# 4. Polygon demo executable
add_executable(polygon_demo demos/polygon_demo.cpp)
target_link_libraries(polygon_demo PRIVATE optibench_lib)

# 3. Polygon test executable
add_executable(test_polygon tests/test_polygon.cpp)
target_link_libraries(test_polygon PRIVATE optibench_lib)

# 3. polygon clipping/filling demo executable
add_executable(demo_clip_fill demos/clip_fill.cpp)
target_link_libraries(demo_clip_fill PRIVATE optibench_lib)
