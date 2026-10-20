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

# --- Entry Point Target

# Main Application Target
add_executable(OptiBench main.cpp)
target_link_libraries(OptiBench PRIVATE optibench_lib)

# --- Test Targets

# 1. Framebuffer test executable
add_executable(test_framebuffer tests/test_framebuffer.cpp)
target_link_libraries(test_framebuffer PRIVATE optibench_lib)

# 2. Circles/Ellipses test executable
add_executable(test_circle_ellipse tests/test_circle_ellipse.cpp)
target_link_libraries(test_circle_ellipse PRIVATE optibench_lib)

# 3. Line test executable
add_executable(test_line tests/test_line.cpp)
target_link_libraries(test_line PRIVATE optibench_lib)

# 4. Polygon clipping/filling test executable
add_executable(test_clip_fill tests/test_clip_fill.cpp)
target_link_libraries(test_clip_fill PRIVATE optibench_lib)

# 5. Polygon test executable
add_executable(test_polygon tests/test_polygon.cpp)
target_link_libraries(test_polygon PRIVATE optibench_lib)

# --- Demo Targets

# 1. Initialization script target executable
add_executable(demo_ok_test demos/ok_test.cpp)
target_link_libraries(demo_ok_test PRIVATE optibench_lib)

# 2. circle/ellipse demo executable
add_executable(demo_circle_ellipse demos/circle_ellipse.cpp)
target_link_libraries(demo_circle_ellipse PRIVATE optibench_lib)

# #3. Line demo executable
add_executable(demo_line demos/line_demo.cpp)
target_link_libraries(demo_line PRIVATE optibench_lib)

# 4. polygon clipping/filling demo executable
add_executable(demo_clip_fill demos/clip_fill.cpp)
target_link_libraries(demo_clip_fill PRIVATE optibench_lib)

# 5. Polygon demo executable
add_executable(demo_polygon demos/polygon_demo.cpp)
target_link_libraries(demo_polygon PRIVATE optibench_lib)

add_executable(demo_integrated demos/integrated_demo.cpp)
target_link_libraries(demo_integrated PRIVATE optibench_lib)


# =====================================================================================
# Curves module (Bezier curves & splines) and ctest registration
# Appended block: no lines above this block were modified.
# Run every test with:  ctest --test-dir build --output-on-failure
# =====================================================================================
enable_testing()

# existing module tests, run from the repository root so their output/ paths resolve
foreach(ob_existing_test test_framebuffer test_line test_polygon test_clip_fill test_circle_ellipse)
    add_test(NAME ${ob_existing_test} COMMAND ${ob_existing_test}
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endforeach()

# strict warnings for the curves code only (other modules keep their current flags)
set(OB_CURVES_WARNINGS "")
if(NOT MSVC)
    set(OB_CURVES_WARNINGS -Wall -Wextra -Wpedantic)
endif()

# curves sources are compiled into the shared library
set(OB_CURVES_SOURCES
        src/curves/CurveMath.cpp
        src/curves/Bezier.cpp
        src/curves/Composite.cpp
        src/curves/CubicSpline.cpp
        src/curves/Hermite.cpp
        src/curves/CatmullRom.cpp
        src/curves/BSpline.cpp
        src/curves/CurveEditor.cpp
        src/curves/OpticalCurve.cpp
        src/curves/RayPath.cpp
)
set(OB_CURVES_HEADERS
        include/optibench/curves/CurveMath.h
        include/optibench/curves/Bezier.h
        include/optibench/curves/Composite.h
        include/optibench/curves/CubicSpline.h
        include/optibench/curves/Hermite.h
        include/optibench/curves/CatmullRom.h
        include/optibench/curves/BSpline.h
        include/optibench/curves/CurveEditor.h
        include/optibench/curves/OpticalCurve.h
        include/optibench/curves/RayPath.h
)
target_sources(optibench_lib PRIVATE ${OB_CURVES_SOURCES} ${OB_CURVES_HEADERS})
set_source_files_properties(${OB_CURVES_SOURCES} PROPERTIES COMPILE_OPTIONS "${OB_CURVES_WARNINGS}")

# adds one curves test executable from tests/curves/<name>.cpp and registers it with ctest
function(ob_add_curves_test name)
    add_executable(${name} tests/curves/${name}.cpp)
    target_link_libraries(${name} PRIVATE optibench_lib)
    target_compile_options(${name} PRIVATE ${OB_CURVES_WARNINGS})
    add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endfunction()

ob_add_curves_test(test_curves_harness)
ob_add_curves_test(test_curves_bezier)
ob_add_curves_test(test_curves_composite)
ob_add_curves_test(test_curves_splines)
ob_add_curves_test(test_curves_editor)
