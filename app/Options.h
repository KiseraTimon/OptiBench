#pragma once
#include <optional>
#include <string>
#include <vector>

namespace ob::app {
    // command-line options of the OptiBench application
    struct Options {
        std::string outDir = "output";      // where images are written
        double duration = 4.0;              // seconds of animation
        double fps = 15.0;
        double stillTime = 2.0;             // the frame saved as final_scene.bmp
        bool writeFrames = true;            // every frame to <out>/frames/final_####.bmp
        bool writeSheet = true;             // the contact sheet
        bool showBounds = true;             // broad-phase boxes (corner brackets)
        bool quiet = false;
        bool help = false;
        bool version = false;
    };

    // parse result: options, or an error message for the user
    struct ParseResult {
        Options options;
        std::optional<std::string> error;
    };

    ParseResult parseOptions(const std::vector<std::string>& args);    // args without the program name
    std::string usage(const std::string& program);
}
