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

### Transform & Optics Math
```bash
.\build\test_mat3.exe        # 3x3 homogeneous matrix toolkit
```

```bash
.\build\test_refraction.exe  # Snell's law + total internal reflection
```

`Mat3` (`include/optibench/Mat3.h`) builds translate, scale, shear, rotate (about the origin or a pivot),
reflect (across the axes or any line) and `composite(t, r, s)` = T·R·S matrices. `Mat3::fromTransform`
gives the same matrix as an element's `Transform2D`, and `transformPoints` maps a whole polygon's vertices.
`refraction.h` adds `criticalAngle`, `refractOrReflect` and `refractAtSegment`, which bend a ray by
Snell's law and fall back to reflection beyond the critical angle.

### Animation
```bash
.\build\test_timeline.exe   # lerp, easing (quadratic + cubic Bezier), keyframe tracks, motion paths, clock
.\build\test_animator.exe   # double buffering and the animation loop driving a Scene
```

```bash
.\build\demo_animation.exe  # demo file
```

The demo animates the periscope bench for 4 seconds at 15 fps and writes every frame to
`output/frames/anim_####.bmp` (not committed), plus `output/animation_demo.bmp`, a contact sheet of six frames:
the top mirror swings on keyframes with ease-in-out, the laser slides out and back, the lens glides along a
cubic Bezier **motion path** (dim arc) timed by a cubic Bezier **easing curve** (graph bottom-right, the yellow dot
is "now"), and the prism turns with an overshooting Bezier ease. The beam is re-traced every frame.
Stitch the frames into a video with `ffmpeg -framerate 15 -i output/frames/anim_%04d.bmp -pix_fmt yuv420p output/animation.mp4`.
Design notes, the math, usage and limitations are in **[docs/animation.md](docs/animation.md)**.

The engine (`Timeline.h`, `DoubleBuffer.h`, `Animator.h`):

| Technique | Where |
|---|---|
| Linear interpolation (tweening) | `lerp(a, b, u)` for numbers and `Vec2` |
| Easing | `easeIn` (u²), `easeOut`, `easeInOut`, or any `double(double)` callable |
| Bezier easing curves | `CubicBezierEasing(x1, y1, x2, y2)`: cubic through (0,0) and (1,1); solves x(s) = u by Newton + bisection, returns y(s). Presets `ease()`, `easeIn()`, `easeOut()`, `easeInOut()`, `backOut()` |
| Keyframes | `Track` (doubles) / `Vec2Track` (points): `add(time, value, easing)`, `sample(t)`; the easing shapes the segment that starts at that keyframe, values clamp outside the keyframes |
| Motion paths | `MotionPath(curve, t0, t1, easing, constantSpeed)`: moves along any Bezier / composite / Catmull-Rom curve, at constant speed via arc-length reparameterization; `heading(t)` gives the direction of travel |
| Delta time | `Clock::advance(dt)`: animation is sampled at elapsed seconds, never at frame numbers; pause, speed, `loopTime`, `pingPongTime`; `RealTimeSource::lap()` for a live loop |
| Double buffering | `DoubleBuffer`: draw into `back()`, `swap()` flips an index in O(1), only `front()` is shown or saved |
| Animation loop | `Animator`: per step advance clock → sample bindings → `scene.render(back)` → overlays → swap; `run(duration, fps, present)` renders offline |

```cpp
Animator anim(scene, buffers);
anim.animateRotation("mirrorTop", Track{}.add(0.0, a0, easeInOut).add(1.2, a1));
anim.animatePath("lens", MotionPath(bezier, 0.0, 4.0, CubicBezierEasing::easeInOut()));
anim.run(4.0, 15.0, [](const Framebuffer& front, const FrameInfo& f) { return front.saveBMP(...); });
```

## Current Progress
- [x] Project skeleton, build system, framebuffer
- [x] Lines
- [x] Polygons
- [x] Clipping/filling
- [x] Circles/ellipses
- [x] Curves
- [x] Integrated demo scene
- [x] Animation (keyframes, easing, Bezier easing & motion paths, double buffering)

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
