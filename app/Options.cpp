#include "Options.h"

#include <charconv>
#include <cmath>
#include <cstdio>

namespace ob::app {
    namespace {
        std::string format(double v) {
            char buf[32];
            std::snprintf(buf, sizeof buf, "%g", v);
            return buf;
        }

        std::optional<double> toNumber(const std::string& s) {
            double v = 0.0;
            const char* end = s.data() + s.size();
            const auto [ptr, ec] = std::from_chars(s.data(), end, v);
            if (ec != std::errc{} || ptr != end || !std::isfinite(v)) return std::nullopt;
            return v;
        }
    }

    ParseResult parseOptions(const std::vector<std::string>& args) {
        ParseResult r;
        Options& o = r.options;
        auto fail = [&](std::string message) {
            r.error = std::move(message);
            return r;
        };

        for (std::size_t i = 0; i < args.size(); ++i) {
            const std::string& a = args[i];
            // options that take a value: "--fps 30" or "--fps=30"
            std::string key = a, value;
            bool inlineValue = false;
            if (const auto eq = a.find('='); a.rfind("--", 0) == 0 && eq != std::string::npos) {
                key = a.substr(0, eq);
                value = a.substr(eq + 1);
                inlineValue = true;
            }
            auto takeValue = [&]() -> bool {
                if (inlineValue) return true;
                if (i + 1 >= args.size()) return false;
                value = args[++i];
                return true;
            };
            auto takeNumber = [&](double& target, double min, double max) -> std::optional<std::string> {
                if (!takeValue()) return key + " needs a value";
                const auto v = toNumber(value);
                if (!v || *v < min || *v > max)
                    return key + " must be a number from " + format(min) + " to " + format(max) + " (got '" + value + "')";
                target = *v;
                return std::nullopt;
            };

            if (key == "-h" || key == "--help") o.help = true;
            else if (key == "--version") o.version = true;
            else if (key == "-q" || key == "--quiet") o.quiet = true;
            else if (key == "--no-frames") o.writeFrames = false;
            else if (key == "--no-sheet") o.writeSheet = false;
            else if (key == "--no-bounds") o.showBounds = false;
            else if (key == "-o" || key == "--out") {
                if (!takeValue() || value.empty()) return fail(key + " needs a folder");
                o.outDir = value;
            }
            else if (key == "--fps") { if (auto e = takeNumber(o.fps, 1.0, 240.0)) return fail(*e); }
            else if (key == "--duration") { if (auto e = takeNumber(o.duration, 0.0, 60.0)) return fail(*e); }
            else if (key == "--still") { if (auto e = takeNumber(o.stillTime, 0.0, 60.0)) return fail(*e); }
            else return fail("unknown option '" + a + "' (try --help)");
        }
        if (o.stillTime > o.duration) o.stillTime = o.duration;
        return r;
    }

    std::string usage(const std::string& program) {
        return "OptiBench: a 2D optical bench, ray traced and rendered frame by frame.\n"
               "\n"
               "Usage: " + program + " [options]\n"
               "\n"
               "  -o, --out DIR      folder for the images (default: output)\n"
               "      --duration S   seconds of animation, 0-60 (default: 4; the animation loops every 4 s)\n"
               "      --fps N        frames per second, 1-240 (default: 15)\n"
               "      --still T      time of the frame saved as final_scene.bmp (default: 2)\n"
               "      --no-frames    do not write every frame to DIR/frames/\n"
               "      --no-sheet     do not write the contact sheet\n"
               "      --no-bounds    hide the broad-phase bounding boxes\n"
               "  -q, --quiet        print nothing unless something fails\n"
               "  -h, --help         show this help\n"
               "      --version      show the version\n"
               "\n"
               "Writes DIR/final_scene.bmp, DIR/final_scene_sheet.bmp and DIR/frames/final_####.bmp.\n"
               "Exit status is 0 when every frame was written and passed its checks (no overlapping\n"
               "elements, every total internal reflection at or past the critical angle).\n";
    }
}
