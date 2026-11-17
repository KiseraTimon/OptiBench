#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "optibench/Framebuffer.h"

/**
 * A minimal native window for the live viewer: shows a Framebuffer and reports input.
 *
 * Written directly against the operating system (Xlib on Linux, Win32 GDI on Windows), so the project
 * still needs no third-party libraries. The window has a fixed client size equal to the framebuffer;
 * pointer positions are reported in framebuffer pixels.
 */
namespace ob::live {
    // keys the viewer uses; printable keys are reported as their lowercase ASCII code
    namespace keys {
        constexpr int Escape = 27;
        constexpr int Space = ' ';
        constexpr int Delete = 127;
        constexpr int Left = 0x1001, Right = 0x1002, Up = 0x1003, Down = 0x1004;
        constexpr int F1 = 0x1010;
    }

    struct Event {
        enum class Type { Quit, Key, MouseDown, MouseUp, MouseMove, Wheel, Redraw };
        Type type = Type::Redraw;
        int key = 0;            // Key
        int button = 0;         // MouseDown / MouseUp: 1 left, 2 middle, 3 right
        int wheel = 0;          // Wheel: +1 away from the user, -1 towards
        int x = 0, y = 0;       // pointer position (framebuffer pixels)
        bool shift = false, ctrl = false;
    };

    class Window {
    public:
        // @throws std::runtime_error if no window can be opened (e.g. no display)
        Window(const std::string& title, int width, int height);
        ~Window();
        Window(const Window&) = delete;
        Window& operator=(const Window&) = delete;

        // the next pending event, without blocking; false when there is none
        bool poll(Event& e);

        // copies the framebuffer to the window (it must be width x height)
        void present(const Framebuffer& fb);

        void setTitle(const std::string& title);

        struct Impl;        // the platform's state (public only so the Win32 window procedure can reach it)

    private:
        std::unique_ptr<Impl> impl_;
    };
}
