#!/usr/bin/env python3
"""
Turns the OptiBench frames into an animated GIF (for the README, or to share) without ffmpeg.

    ./build/OptiBench                       # writes output/frames/final_####.bmp
    pip install pillow
    python3 tools/make_gif.py               # writes docs/images/final_scene.gif

Options: --frames DIR  --out FILE  --width PX (default 640)  --fps N (default 15)
"""
import argparse
import glob
import os
import sys

from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", default=os.path.join(ROOT, "output", "frames"))
    ap.add_argument("--out", default=os.path.join(ROOT, "docs", "images", "final_scene.gif"))
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--fps", type=float, default=15.0)
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.frames, "final_*.bmp")))
    if not paths:
        sys.exit(f"no frames in {args.frames}: run ./build/OptiBench first")

    frames = []
    for p in paths:
        im = Image.open(p).convert("RGB")
        h = round(im.height * args.width / im.width)
        frames.append(im.resize((args.width, h), Image.LANCZOS))

    # one shared palette from a few frames keeps colours stable from frame to frame (no flicker)
    sample = Image.new("RGB", (args.width, frames[0].height * 3))
    for i, f in enumerate(frames[:: max(1, len(frames) // 3)][:3]):
        sample.paste(f, (0, i * f.height))
    palette = sample.quantize(colors=255, method=Image.Quantize.MEDIANCUT)
    gif = [f.quantize(palette=palette, dither=Image.Dither.FLOYDSTEINBERG) for f in frames]

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    gif[0].save(args.out, save_all=True, append_images=gif[1:], duration=round(1000 / args.fps), loop=0, optimize=True)
    print(f"wrote {os.path.relpath(args.out, ROOT)}: {len(gif)} frames, {os.path.getsize(args.out) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
