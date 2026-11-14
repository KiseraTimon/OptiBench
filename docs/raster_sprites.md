# Raster operations, sprites, and lighting

`raster_ops.h` provides clipped framebuffer compositing. `Color` and `Framebuffer` remain RGB-only;
transparency is represented by a color key or a separate mask.

## Blitting

```cpp
blit(source, {0, 0, source.width(), source.height()}, destination, x, y, BlitMode::Copy);
blit(source, sourceRect, destination, x, y, BlitMode::KeepDestination); // skips black pixels
```

`Copy` replaces destination pixels. `KeepDestination` skips pixels matching `BlitOptions::colorKey`
(black by default). `And`, `Or`, and `Xor` apply bitwise operations to each RGB channel. All blits
clip against source and destination bounds; self-blits snapshot the source region, so overlapping
copies behave as expected. `Mask` copies only where the corresponding source-coordinate pixel in
`BlitOptions::mask` matches `maskColor` (white by default). A missing mask makes a Mask blit a no-op.

`blendOver(src, dst, alpha)` computes `src * alpha + dst * (1 - alpha)` independently per channel,
clamps alpha to `[0, 1]`, and rounds to the nearest byte. `blendPixel` applies that operation to an
in-bounds framebuffer pixel. This parameter is opacity supplied by the caller, not alpha stored in
the source pixel.

## Sprites

`Sprite` owns a framebuffer image and an optional color key (black by default). It draws at a pixel
position with clipping, using KeepDestination by default. Pass `std::nullopt` to disable keyed
transparency. `SpriteSheet` owns a framebuffer of fixed-size frames in row-major order; incomplete
right and bottom edge frames are ignored, and invalid frame indices draw nothing.

## Beam lighting

Scene beam glow and screen-hit glow use `blendPixel` with a distance-based opacity falloff. The
existing ray tracing, clipping, framebuffer, and beam-core rendering layers remain unchanged.

Build and run `demo_raster_sprites` from the repository root to write
`output/raster_sprites_demo.bmp`.
