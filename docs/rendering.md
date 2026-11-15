# Rendering: spectral colour, HDR light and text

Headers: `include/optibench/render/` (`Spectrum.h`, `LightBuffer.h`, `Text.h`) · Sources: `src/render/` ·
Tests: `tests/render/` · Used by: the OptiBench application (`app/Studio.cpp`)

Dispersion, the physics half of this work, lives in `Material.h` (`Material::indexAt`) and `Tracer.h`
(`traceBeam(scene, ray, maxSteps, wavelengthNm)`).

---

## 1. Why a renderer on top of the raster modules

The phase-1 modules draw hard, single-colour lines and fills into an 8-bit framebuffer. That is the right
foundation, but light does not behave like ink:

* **Light adds.** Two beams crossing are brighter than either, and 61 colours of a split white beam sum to
  white. Adding 8-bit sRGB values gives the wrong answer, because sRGB is gamma-encoded. Light has to be added
  in **linear** space.
* **Light has a dynamic range.** A laser core is far brighter than anything a screen can show. An HDR buffer
  keeps the real values, and a **tone map** compresses them at the end.
* **Bright light glows.** Cameras and eyes scatter bright light into its surroundings. **Bloom** imitates this,
  and it is what makes a beam read as light rather than as a line.

So the renderer has one rule: draw the scene into the framebuffer, accumulate light in a float
`LightBuffer`, and combine the two in linear space exactly once (`composite`).

## 2. Colour

### sRGB
`srgbToLinear` and `linearToSrgb` implement the IEC 61966-2-1 transfer curves. Decoding 8-bit values uses a
256-entry table. Encoding uses a 4096-entry table, which is finer than 8 bits near black, where the curve is
steep. A test checks that every 8-bit value survives a round trip exactly.

### The colour of a wavelength
`cieXyz(nm)` evaluates the CIE 1931 2° colour matching functions with the multi-lobe Gaussian fit of Wyman,
Sloan and Shirley [1]:

    g(λ; μ, σ₁, σ₂) = exp(-½((λ - μ)/σ)²),   σ = σ₁ if λ < μ else σ₂
    x̄ = 1.056 g(599.8, 37.9, 31.0) + 0.362 g(442.0, 16.0, 26.7) - 0.065 g(501.1, 20.4, 26.2)
    ȳ = 0.821 g(568.8, 46.9, 40.5) + 0.286 g(530.9, 16.3, 31.1)
    z̄ = 1.217 g(437.0, 11.8, 36.0) + 0.681 g(459.0, 26.0, 13.8)

XYZ is converted to linear sRGB with the standard D65 matrix. Pure spectral colours lie **outside** the sRGB
gamut, so some components come out negative. They are clipped to zero, which keeps the colours as saturated
as the display allows. Outside 360–830 nm the colour is black.

`laserColor(nm)` normalises the colour so its largest component is 1, giving the colour of a laser line.
`whiteLightSamples(n)` spreads `n` wavelengths over 400–700 nm and **white-balances** them: each channel is
scaled so the samples sum to exactly (1, 1, 1). Wherever the rays overlap they look white, and wherever glass
separates them a spectrum appears.

## 3. Dispersion

Glass catalogues give two numbers per glass: n<sub>d</sub>, the index at the helium d line (587.56 nm), and
the **Abbe number**

    V_d = (n_d - 1) / (n_F - n_C)          F = 486.13 nm, C = 656.27 nm

A low V means strong dispersion. `Material::indexAt(λ)` fits **Cauchy's equation** n(λ) = A + B/λ² to that
pair:

    B = (n_d - 1) / V_d / (1/λ_F² - 1/λ_C²)
    n(λ) = n_d + B (1/λ² - 1/λ_d²)          (λ in µm, so n(λ_d) = n_d exactly)

The tests check that n(λ<sub>d</sub>) = n<sub>d</sub> and that (n<sub>d</sub>−1)/(n<sub>F</sub>−n<sub>C</sub>) = V
both hold exactly. For N-BK7 the model gives n(404.66 nm) = 1.53034, against 1.53024 in the Schott data sheet [2].

| Preset | n<sub>d</sub> | V<sub>d</sub> | n(400) − n(700) |
|---|---|---|---|
| `crownBK7()` | 1.5168 | 64.17 | 0.018 |
| `flintF2()` | 1.6200 | 36.37 | 0.038 |
| `denseFlintSF11()` | 1.7847 | 25.68 | 0.067 |
| `denseFlintSF66()` | 1.9229 | 20.88 | 0.097 |

`traceBeam` takes a wavelength and uses `indexAt` at every glass surface, so Snell's law and the critical
angle are both per colour. At the design wavelength, which is the default, every glass has exactly its old
`refractiveIndex`. The existing scenes and tests are therefore unchanged: their images were compared byte
for byte.

**Designing the spectrometer.** An equilateral SF11 prism is the classic choice. An N-SF66 prism cannot be
used at 60°, because its critical angle at 400 nm (30.0°) leaves no margin over the 30° internal angle
that minimum deviation needs. The bench's turning mirror and prism were placed by a sweep. It
looked for the widest spectrum that stays on the screen, avoids total internal reflection, and holds over
the notch's whole range of rotation with ±0.015 rad of slack. The test `test_app_bench` re-checks every frame.

## 4. The light buffer

### Gaussian beams
`addBeam(a, b, peak, σ)` adds, at every pixel near the segment,

    w = ends(t) · exp(-d⊥² / 2σ²)

Here d⊥ is the distance from the beam's centre line, and `ends` is a one-pixel linear ramp centred on each
end point. Where two segments share an end point their ramps are complementary (they sum to 1), so a beam
chained through many surfaces stays uniform with no hot spots at the joins. A test checks this. The light
per unit length is `peak · σ · √(2π)` at any angle. The loop walks the beam's major axis and visits only a
band ±3.5σ wide around it, so the cost is proportional to the beam's length, not to its bounding box.

### Bloom
1. Halve the resolution (2×2 average).
2. Blur two copies: a tight glow (radius 3) and a wide halo (radius 14).
3. Mix them, then sample bilinearly at full resolution.

Each blur is **three box passes** in each direction. By the central limit theorem this is close to a
Gaussian, and a sliding window makes it O(1) per pixel whatever the radius. Blurring conserves the light
(a test checks this to 0.1%).

### Tone mapping
Each channel goes through `1 − e^(−x)`. That is linear for dim light (1 − e^(−x) ≈ x), never clips, and
rolls off smoothly. Light brighter than 1 then **bleeds into white**:

    excess = max(0, max(r, g, b) - 1);   bleed = whiteCore · (1 - e^-excess)

This is why a saturated red core looks white-hot with a red glow, as a laser does on camera. A vignette
(1 − k·r<sup>2.5</sup>) darkens the corners slightly.

## 5. Text

`tools/generate_font.py` rasterises DejaVu Sans Mono at 12, 14 and 20 px (bold) into 4-bit coverage tables
in `src/render/FontData.inc`. The output is committed, so the build does not need Python. `drawText` decodes
UTF-8 and blends each glyph's coverage over the framebuffer in linear light. Besides ASCII it supports
`° θ λ · → × µ ≥ ≤ Δ ✓ ✗ ± – ′`, and unknown characters draw as `?`. `drawLabel` puts a translucent chip
behind the text so it stays readable over beams.

## 6. How the Studio uses it

`app/Studio.cpp` renders each frame of the final scene in this order:

1. **Backdrop**, drawn once and copied each frame: the optical breadboard (a 25 px hole grid), rulers, the
   header with a spectral accent line, and the HUD panels.
2. **Elements**, shaded per pixel. Coverage comes from `geom::signedDistance`, so every edge is
   anti-aliased at any angle.
   - Glass is tinted by its Abbe number (crowns cyan, flints violet), with a bright rim and a top sheen.
   - Mirrors are brushed metal with a silver face; the dish is hatched on its back.
   - Screens are matte, with ruler ticks.
   - Laser housings have cooling fins and a colour band.
3. **Light.** Every traced ray goes into the light buffer, with a wider halo inside glass. Landing spots,
   indicator LEDs and photon glows are added too.
4. **`composite`**.
5. **Overlays**:
   - event markers (refraction, reflection and TIR rings)
   - lens and dish foci, and the motion path
   - corner brackets for the broad-phase boxes
   - the XOR selection box and sprite photons
   - labels with live readouts, and the spectrum's wavelength scale, placed where 400, 450 … 700 nm actually land
6. **HUD**: a `Mat3` minimap, per-beam paths and counts, the checks, and a keyframe timeline with a playhead.

`Animator::setRenderer` lets the Studio replace the default `Scene::render`, so nothing is drawn twice.

## References

1. C. Wyman, P.-P. Sloan, P. Shirley. *Simple Analytic Approximations to the CIE XYZ Color Matching
   Functions.* Journal of Computer Graphics Techniques 2(2), 2013.
2. SCHOTT AG, *Optical Glass Data Sheets* (N-BK7, F2, SF11, N-SF66).
3. IEC 61966-2-1:1999, *Default RGB colour space: sRGB.*
4. E. Hecht, *Optics*, 5th ed., §3.5 (dispersion) and §5.5 (prisms).
5. W. Jarosz, *Fast Image Convolutions* (SIGGRAPH 2001 course notes): repeated box filters approximate a
   Gaussian.
