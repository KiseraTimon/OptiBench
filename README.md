# OptiBench

> Interactive optics simulation.

**Status:** Checkpoint 1

## Description
OptiBench simulates how light travels through an optical bench: laser rays reflect off mirrors,
bend through lenses and prisms, and eventually split into colours. Checkpoint 1 builds the
2D foundation.

## Group Members

- Sean Mudibo
- Cyprian Kamau
- Sharon Njogorio
- Amy Mugeni
- Kisera Timon

## Technologies / Tools
- VS Code or CLion IDE
- C++20
- CMake, g++ / clang / MSVC
- Git + GitHub
- Output: `.bmp`

## Prerequisites
This project requires MSYS2's GCC and cmake to run. A comprehensive guide has been provided **[here](references/prerequisites_guide.pdf)**

## How to Compile and Run
Requires a C++17 compiler and CMake 3.16+.

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

```bash
./build/ok_test.exe
```

Running the okay test should instantiate the Framebuffer to make a gradient image. If this is successful, the program works okay

### Polygon clipping & filling
```bash
./build/test_clip_fill.exe
./build/demo_clip_fill.exe
```
The demo writes `output/clip_fill_demo.bmp`: a prism (scanline fill), a light beam clipped to the bench
viewport and to the prism (Sutherland-Hodgman), laser rays trimmed to the viewport (Cohen-Sutherland),
and a laser source and mirror coloured with flood/boundary fill. Run it from the repository root.

## Current Progress
- [x] Project skeleton, build system, framebuffer
- [x] Lines
- [x] Polygons
- [x] Clipping/filling
- [x] Circles/ellipses
- [ ] Curves
- [ ] Integrated demo scene

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
