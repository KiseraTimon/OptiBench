# OptiBench

> Interactive optics simulation.

**Status:** Checkpoint 1

## Description
OptiBench simulates how light travels through an optical bench: laser rays reflect off mirrors,
bend through lenses and prisms, and eventually split into colours. Checkpoint 1 builds the
2D foundation.

## Group Members

- Sean Mudibo - Polygon Clipping
- Cyprian Kamau - Curves
- Sharon Njogorio - Lines
- Amy Mugeni - Polygons
- Kisera Timon - Circles & Ellipses

## Technologies / Tools
- VS Code or CLion IDE
- C++20
- CMake, g++ / clang / MSVC
- Git + GitHub
- Output: `.bmp`

## Prerequisites
This project requires MSYS2's GCC and cmake to run. A comprehensive guide has been provided **[here](references/prerequisites_guide.pdf)**

## How to Compile and Run
Requires a C++20 compiler and CMake 3.20+.

```bash
git clone https://github.com/KiseraTimon/OptiBench.git
```

```bash
cd OptiBench
```

```bash
cmake -S . -B build
```

```bash
cmake --build build
```

### Setup

```bash
.\build\ok_test.exe
```

Running the okay test should instantiate the Framebuffer to make a gradient image. If this is successful, the program works okay

### Running all tests
```bash
ctest --test-dir build --output-on-failure
```

Runs every module's test executable from the repository root.

### Polygon clipping & filling
```bash
.\build\test_clip_fill.exe  # test file
```

```bash
.\build\demo_clip_fill.exe  # demo file
```

The demo writes `output/clip_fill_demo.bmp`: a prism (scanline fill), a light beam clipped to the bench
viewport and to the prism (Sutherland-Hodgman), laser rays trimmed to the viewport (Cohen-Sutherland),
and a laser source and mirror coloured with flood/boundary fill. Run it from the repository root.

### Lines
```bash
.\build\test_line.exe  # test file
```

```bash
.\build\demo_line.exe  # demo file
```

The demo writes `output/line_demo.bmp`: Features three distinct optical paths—red, green, and
blue—bouncing jaggedly in a zig-zag pattern between parallel white boundaries (mirrors) before
hitting a yellow vertical line (detector) on the right

### Polygons
```bash
.\build\test_polygon.exe  # test file
```

```bash
.\build\demo_polygon.exe  # demo file
```

The demo writes `output/polygon_demo.bmp`: Presents a simple scene with a hollow cyan rectangle on
the left and a hollow yellow triangle on the right, mapped on a dark background grid with white dots
marking the vertices of both shapes

### Circles & Ellipses
```bash
.\build\test_circle_ellipse.exe  # test file
```

```bash
.\build\demo_circle_ellipse.exe  # demo file
```

The demo writes `output/circle_ellipse_demo.bmp`: Displays a large blue circular lens bordered
in cyan with a red laser ray approaching from the left, refracting inside the lens,
and exiting out the other side as a white/yellow line.
The scene also includes a yellow ellipse in the top right and a series of
concentric grey circles in the bottom left

### Curves
```bash
.\build\test_curves_bezier.exe  # test files: test_curves_*.exe (bezier, composite, splines, editor, optics, render, nurbs)
```

```bash
.\build\demo_curves.exe  # demo file
```

The demo writes `output/curves_demo.bmp`: four panels showing the anatomy of a cubic Bézier curve
(de Casteljau construction, tangent and normal, tight bounding box, adaptive flattening points), four
splines through the same points (Catmull-Rom uniform and centripetal, natural cubic, clamped B-spline),
parallel laser rays refracting through a lens whose faces are Bézier curves, and a parabolic mirror
focusing rays above a gradient-index fibre that guides rays along smooth curved paths. Design notes,
usage and references are in **[docs/curves.md](docs/curves.md)**.

## Current Progress
- [x] Project skeleton, build system, framebuffer
- [x] Lines
- [x] Polygons
- [x] Clipping/filling
- [x] Circles/ellipses
- [x] Curves
- [x] Integrated demo scene

## Known Limitations
- 2D Only
- Windows-biased

## Repository Layout
```
include/optibench/   headers (one header per module)
src/                 implementations
demos/               one runnable illustration of a module + integrated demo
tests/               automated checks
docs/                notes and screenshots
```
