# OptiBench Live

`OptiBench_live` is the interactive version of the final scene (`app/live/`). It uses the bench
(`buildBench`), the animation (`animateBench`) and the `Studio` renderer from the offline application. The only
differences are that it shows frames in a window as they are made, and that the mouse and keyboard can change the bench.

## How it works

```
loop:
  poll window events -> handle()        drag / rotate / path edits / keys
  advance(dt * speed)                   Animator: bindings sampled at t, then the live updates (below)
  Studio::render(frame)                 trace every ray, light, bloom, HUD (unchanged from the offline app)
  overlay                               selection outline, path handles, status, help
  Window::present(frame)                Xlib XPutImage / Win32 SetDIBitsToDevice
```

* **Placed elements.** After the animation's bindings run, an `Animator::addUpdate` re-applies the transforms
  of every element the user has moved, so a dragged mirror stays where it was put while the rest of the bench
  keeps animating. `Del` hands it back to the animation.
* **Overlap guard (geometry module).** Every move or rotation is checked with `Scene::overlappingPairs()` (SAT on
  the elements' outlines). A move that would overlap another element is refused, and the outline turns red, naming
  the element in the way. The tracer assumes glass never overlaps, so this keeps the optics valid.
* **Lens path (curves module).** `P` shows the lens's motion path as a `CurveEditor` over its Bezier control
  points (hit-testing, transactional drags, `insertNear`, `remove`). Every edit rebuilds the `MotionPath` with the
  same timing as `Bench.cpp` (constant speed, cubic-Bezier ease-in-out), and the lens follows the new curve.
* **Speed.** The compositor (`render::composite`, the bloom blurs) and the beam rasterisation in
  `Studio::addLight` share their rows between CPU threads (`render/Parallel.h`). Each thread writes only its own
  rows or band, so the frames match the serial renderer (at most one colour level on a rare pixel, from float
  rounding at a band edge). A frame takes about 55 ms on 8 threads, against 135 ms before.

## Checking it without a window

```bash
./build/OptiBench_live --self-test output/live_selftest
```

This runs scripted drags, wheel turns, a refused overlap, path edits and key presses through the same
`handle()` the window uses. It verifies each result and writes screenshots. It needs no display and runs in ctest
as `live_selftest`.
