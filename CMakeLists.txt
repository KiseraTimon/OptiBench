cmake_minimum_required(VERSION 3.20)
project(OptiBench VERSION 1.0.0 DESCRIPTION "2D optical bench: spectral ray tracing and raster graphics" LANGUAGES CXX)

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
        include/optibench/Sprite.h
        src/Sprite.cpp
        src/raster_ops.cpp

        # scene architecture
        include/optibench/Transform2D.h
        include/optibench/Material.h
        include/optibench/OpticalElement.h
        include/optibench/Elements.h
        include/optibench/Scene.h
        include/optibench/Tracer.h
        include/optibench/optics.h
        src/Elements.cpp
        src/Scene.cpp
        src/Tracer.cpp
        src/optics.cpp

        # contracts for next modules
        include/optibench/Mat3.h
        include/optibench/raster_ops.h
        include/optibench/Timeline.h
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

# 6. Scene architecture test executable
add_executable(test_scene tests/test_scene.cpp)
target_link_libraries(test_scene PRIVATE optibench_lib)


# 7. Ellipse additions test executable
add_executable(test_ellipse_extras tests/test_ellipse_extras.cpp)
target_link_libraries(test_ellipse_extras PRIVATE optibench_lib)

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

add_executable(demo_raster_sprites demos/raster_sprites_demo.cpp)
target_link_libraries(demo_raster_sprites PRIVATE optibench_lib)


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
        src/curves/CurveRender.cpp
        src/curves/Nurbs.cpp
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
        include/optibench/curves/CurveRender.h
        include/optibench/curves/Nurbs.h
        include/optibench/curves/Curves.h
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
ob_add_curves_test(test_curves_optics)
ob_add_curves_test(test_curves_render)
ob_add_curves_test(test_curves_nurbs)

# curves demo: writes output/curves_demo.bmp (run from the repository root)
add_executable(demo_curves demos/curves_demo.cpp)
target_link_libraries(demo_curves PRIVATE optibench_lib)
target_compile_options(demo_curves PRIVATE ${OB_CURVES_WARNINGS})


# =====================================================================================
# Transform & optics maths module (Mat3 toolkit, Snell refraction with TIR fallback)
# Appended block: no lines above this block were modified.
# Run its tests with:  ctest --test-dir build -R "test_mat3|test_refraction" --output-on-failure
# =====================================================================================
set(OB_TRANSFORM_HEADERS
        include/optibench/refraction.h
)
set(OB_TRANSFORM_SOURCES
        src/Mat3.cpp
        src/refraction.cpp
)
target_sources(optibench_lib PRIVATE ${OB_TRANSFORM_SOURCES} ${OB_TRANSFORM_HEADERS})

# adds one transform/optics test executable from tests/<name>.cpp and registers it with ctest
function(ob_add_transform_test name)
    add_executable(${name} tests/${name}.cpp)
    target_link_libraries(${name} PRIVATE optibench_lib)
    add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endfunction()

ob_add_transform_test(test_mat3)
ob_add_transform_test(test_refraction)


# =====================================================================================
# Animation module (delta time, keyframes, easing incl. cubic Bezier, motion paths,
# double buffering, animation loop)
# Appended block: no lines above this block were modified.
# Run its tests with:  ctest --test-dir build -R "test_timeline|test_animator" --output-on-failure
# =====================================================================================
set(OB_ANIMATION_HEADERS
        include/optibench/DoubleBuffer.h
        include/optibench/Animator.h
)
set(OB_ANIMATION_SOURCES
        src/Timeline.cpp
        src/Animator.cpp
)
target_sources(optibench_lib PRIVATE ${OB_ANIMATION_SOURCES} ${OB_ANIMATION_HEADERS})

# strict warnings for the animation code (same flags as the curves module)
set_source_files_properties(${OB_ANIMATION_SOURCES} PROPERTIES COMPILE_OPTIONS "${OB_CURVES_WARNINGS}")

# adds one animation test executable from tests/<name>.cpp and registers it with ctest
function(ob_add_animation_test name)
    add_executable(${name} tests/${name}.cpp)
    target_link_libraries(${name} PRIVATE optibench_lib)
    target_compile_options(${name} PRIVATE ${OB_CURVES_WARNINGS})
    add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endfunction()

ob_add_animation_test(test_timeline)
ob_add_animation_test(test_animator)

# animation demo: writes output/frames/anim_*.bmp and output/animation_demo.bmp (run from the repository root)
add_executable(demo_animation demos/animation_demo.cpp)
target_link_libraries(demo_animation PRIVATE optibench_lib)
target_compile_options(demo_animation PRIVATE ${OB_CURVES_WARNINGS})


# =====================================================================================
# Geometry module (orientation, segment intersection, polygon queries, convex hull,
# triangulation, ray/polygon and ray/box tests, SAT overlap, thick lens shape)
# Appended block: no lines above this block were modified.
# Run its tests with:  ctest --test-dir build -R test_geometry --output-on-failure
# =====================================================================================
set(OB_GEOMETRY_HEADERS
        include/optibench/geometry/Primitives.h
        include/optibench/geometry/PolygonOps.h
        include/optibench/geometry/Intersect.h
        include/optibench/geometry/Overlap.h
        include/optibench/geometry/Lens.h
        include/optibench/geometry/Geometry.h
)
set(OB_GEOMETRY_SOURCES
        src/geometry/Primitives.cpp
        src/geometry/PolygonOps.cpp
        src/geometry/Intersect.cpp
        src/geometry/Overlap.cpp
        src/geometry/Lens.cpp
)
target_sources(optibench_lib PRIVATE ${OB_GEOMETRY_SOURCES} ${OB_GEOMETRY_HEADERS})

# strict warnings for the geometry code (same flags as the curves module)
set_source_files_properties(${OB_GEOMETRY_SOURCES} PROPERTIES COMPILE_OPTIONS "${OB_CURVES_WARNINGS}")

# adds one geometry test executable from tests/geometry/<name>.cpp and registers it with ctest
function(ob_add_geometry_test name)
    add_executable(${name} tests/geometry/${name}.cpp)
    target_link_libraries(${name} PRIVATE optibench_lib)
    target_compile_options(${name} PRIVATE ${OB_CURVES_WARNINGS})
    add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endfunction()

ob_add_geometry_test(test_geometry_primitives)
ob_add_geometry_test(test_geometry_polygon)
ob_add_geometry_test(test_geometry_intersect)
ob_add_geometry_test(test_geometry_overlap)
ob_add_geometry_test(test_geometry_lens)
ob_add_geometry_test(test_geometry_properties)

# geometry demo: writes output/geometry_demo.bmp (run from the repository root)
add_executable(demo_geometry demos/geometry_demo.cpp)
target_link_libraries(demo_geometry PRIVATE optibench_lib)
target_compile_options(demo_geometry PRIVATE ${OB_CURVES_WARNINGS})


# =====================================================================================
# Integration: the combined scene (every module working together)
# Appended block: no lines above this block were modified.
# Run its tests with:  ctest --test-dir build -R "test_integration|test_scene" --output-on-failure
# =====================================================================================

# scene architecture and ellipse tests were built but never registered with ctest
foreach(ob_unregistered_test test_scene test_ellipse_extras)
    add_test(NAME ${ob_unregistered_test} COMMAND ${ob_unregistered_test}
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endforeach()

# scene, elements and tracer working with the geometry, curves and optics modules
add_executable(test_integration tests/test_integration.cpp)
target_link_libraries(test_integration PRIVATE optibench_lib)
target_compile_options(test_integration PRIVATE ${OB_CURVES_WARNINGS})
add_test(NAME test_integration COMMAND test_integration WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# Polygon.h's centroid / contains_point / draw (previously declared but never defined)
add_executable(test_polygon_methods tests/test_polygon_methods.cpp)
target_link_libraries(test_polygon_methods PRIVATE optibench_lib)
target_compile_options(test_polygon_methods PRIVATE ${OB_CURVES_WARNINGS})
add_test(NAME test_polygon_methods COMMAND test_polygon_methods WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# scene architecture demo (had no build target): writes output/scene_demo.bmp
add_executable(demo_scene demos/scene_demo.cpp)
target_link_libraries(demo_scene PRIVATE optibench_lib)

# =====================================================================================
# Rendering (ob::render): spectral colour, HDR light with bloom and tone mapping, text
# =====================================================================================
set(OB_RENDER_HEADERS
    include/optibench/render/Spectrum.h
    include/optibench/render/LightBuffer.h
    include/optibench/render/Text.h
    include/optibench/render/Parallel.h
)
set(OB_RENDER_SOURCES
    src/render/Spectrum.cpp
    src/render/LightBuffer.cpp
    src/render/Text.cpp
    src/render/FontData.inc     # generated by tools/generate_font.py (committed)
)
target_sources(optibench_lib PRIVATE ${OB_RENDER_SOURCES} ${OB_RENDER_HEADERS})

# the compositor shares its rows out between CPU threads (std::thread; pthreads on Linux)
find_package(Threads REQUIRED)
target_link_libraries(optibench_lib PUBLIC Threads::Threads)

function(ob_add_render_test name)
    add_executable(${name} tests/render/${name}.cpp)
    target_link_libraries(${name} PRIVATE optibench_lib)
    target_include_directories(${name} PRIVATE tests)
    target_compile_options(${name} PRIVATE ${OB_CURVES_WARNINGS})
    add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endfunction()

ob_add_render_test(test_render_spectrum)
ob_add_render_test(test_render_light)
ob_add_render_test(test_render_text)
ob_add_render_test(test_dispersion)

# =====================================================================================
# The OptiBench application: the final combined scene (main.cpp + app/)
#   ./build/OptiBench --help
# =====================================================================================
add_library(optibench_app STATIC
    app/Bench.h app/Bench.cpp
    app/Trace.h app/Trace.cpp
    app/Studio.h app/Studio.cpp
    app/Options.h app/Options.cpp
)
target_link_libraries(optibench_app PUBLIC optibench_lib)
target_include_directories(optibench_app PUBLIC app)
target_compile_options(optibench_app PRIVATE ${OB_CURVES_WARNINGS})

target_link_libraries(OptiBench PRIVATE optibench_app)
target_compile_options(OptiBench PRIVATE ${OB_CURVES_WARNINGS})
target_compile_definitions(OptiBench PRIVATE OPTIBENCH_VERSION="${PROJECT_VERSION}")

# the application's bench: layout valid, every beam lands, the spectrum is ordered
add_executable(test_app_bench tests/test_app_bench.cpp)
target_link_libraries(test_app_bench PRIVATE optibench_app)
target_include_directories(test_app_bench PRIVATE tests)
target_compile_options(test_app_bench PRIVATE ${OB_CURVES_WARNINGS})
add_test(NAME test_app_bench COMMAND test_app_bench WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# end-to-end: the application renders a short run into a temporary folder and exits cleanly
add_test(NAME app_smoke
         COMMAND OptiBench --quiet --no-frames --duration 0.4 --out ${CMAKE_BINARY_DIR}/smoke
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# =====================================================================================
# OptiBench Live: the final scene in a window, in real time, editable with the mouse
#   ./build/OptiBench_live        (app/live/: Xlib on Linux, Win32 GDI on Windows; no other libraries)
# Skipped when no window system is found (e.g. a Linux machine without the X11 headers).
# =====================================================================================
option(OPTIBENCH_LIVE "Build the interactive viewer OptiBench_live" ON)
if(OPTIBENCH_LIVE)
    set(OB_LIVE_BACKEND "")
    if(WIN32)
        set(OB_LIVE_BACKEND app/live/Window_win32.cpp)
    elseif(UNIX AND NOT APPLE)
        find_package(X11 QUIET)
        if(X11_FOUND)
            set(OB_LIVE_BACKEND app/live/Window_x11.cpp)
        else()
            message(STATUS "OptiBench_live skipped: X11 development headers not found (e.g. apt install libx11-dev)")
        endif()
    endif()
    if(OB_LIVE_BACKEND)
        add_executable(OptiBench_live app/live/Live.cpp app/live/Window.h ${OB_LIVE_BACKEND})
        target_link_libraries(OptiBench_live PRIVATE optibench_app)
        target_include_directories(OptiBench_live PRIVATE app/live)
        target_compile_options(OptiBench_live PRIVATE ${OB_CURVES_WARNINGS})
        if(WIN32)
            target_link_libraries(OptiBench_live PRIVATE user32 gdi32)
        else()
            target_link_libraries(OptiBench_live PRIVATE X11::X11)
        endif()
        # the interaction, checked headlessly (no display needed): scripted drags, wheel, path edits, keys
        add_test(NAME live_selftest COMMAND OptiBench_live --self-test ${CMAKE_BINARY_DIR}/live_selftest
                 WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    endif()
endif()
