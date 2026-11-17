// Window backend for Linux (and other X11 systems, including WSLg): plain Xlib, no toolkit.
#include "Window.h"

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>

#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace ob::live {
    struct Window::Impl {
        Display* display = nullptr;
        ::Window window = 0;
        GC gc = nullptr;
        XImage* image = nullptr;
        Atom deleteAtom = 0;
        int width = 0, height = 0;
    };

    Window::Window(const std::string& title, int width, int height) : impl_(std::make_unique<Impl>()) {
        Impl& m = *impl_;
        m.width = width;
        m.height = height;
        m.display = XOpenDisplay(nullptr);
        if (!m.display) throw std::runtime_error("cannot open an X display (is DISPLAY set?)");

        const int screen = DefaultScreen(m.display);
        Visual* visual = DefaultVisual(m.display, screen);
        const int depth = DefaultDepth(m.display, screen);
        if (depth < 24 || visual->c_class != TrueColor) {
            XCloseDisplay(m.display);
            throw std::runtime_error("the X display needs a 24-bit TrueColor visual");
        }

        m.window = XCreateSimpleWindow(m.display, RootWindow(m.display, screen), 0, 0, static_cast<unsigned>(width),
                                       static_cast<unsigned>(height), 0, BlackPixel(m.display, screen),
                                       BlackPixel(m.display, screen));
        XSelectInput(m.display, m.window,
                     ExposureMask | KeyPressMask | ButtonPressMask | ButtonReleaseMask | PointerMotionMask |
                         StructureNotifyMask);

        // fixed size: the image is shown 1:1
        XSizeHints* hints = XAllocSizeHints();
        hints->flags = PMinSize | PMaxSize;
        hints->min_width = hints->max_width = width;
        hints->min_height = hints->max_height = height;
        XSetWMNormalHints(m.display, m.window, hints);
        XFree(hints);

        m.deleteAtom = XInternAtom(m.display, "WM_DELETE_WINDOW", False);
        XSetWMProtocols(m.display, m.window, &m.deleteAtom, 1);
        XStoreName(m.display, m.window, title.c_str());
        m.gc = XCreateGC(m.display, m.window, 0, nullptr);

        // 32 bits per pixel, 0x00RRGGBB (XDestroyImage frees the malloc'd pixels)
        char* data = static_cast<char*>(std::malloc(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4));
        if (!data) throw std::runtime_error("out of memory");
        m.image = XCreateImage(m.display, visual, static_cast<unsigned>(depth), ZPixmap, 0, data,
                               static_cast<unsigned>(width), static_cast<unsigned>(height), 32, 0);
        if (!m.image) {
            std::free(data);
            throw std::runtime_error("XCreateImage failed");
        }

        XMapWindow(m.display, m.window);
        XFlush(m.display);
    }

    Window::~Window() {
        Impl& m = *impl_;
        if (m.image) XDestroyImage(m.image);
        if (m.gc) XFreeGC(m.display, m.gc);
        if (m.window) XDestroyWindow(m.display, m.window);
        if (m.display) XCloseDisplay(m.display);
    }

    bool Window::poll(Event& e) {
        Impl& m = *impl_;
        while (XPending(m.display)) {
            XEvent ev;
            XNextEvent(m.display, &ev);
            e = Event{};
            switch (ev.type) {
                case Expose:
                    if (ev.xexpose.count != 0) continue;
                    e.type = Event::Type::Redraw;
                    return true;
                case ClientMessage:
                    if (static_cast<Atom>(ev.xclient.data.l[0]) != m.deleteAtom) continue;
                    e.type = Event::Type::Quit;
                    return true;
                case KeyPress: {
                    const KeySym sym = XLookupKeysym(&ev.xkey, 0);
                    e.type = Event::Type::Key;
                    e.shift = (ev.xkey.state & ShiftMask) != 0;
                    e.ctrl = (ev.xkey.state & ControlMask) != 0;
                    e.x = ev.xkey.x;
                    e.y = ev.xkey.y;
                    switch (sym) {
                        case XK_Escape: e.key = keys::Escape; break;
                        case XK_space: e.key = keys::Space; break;
                        case XK_Tab: case XK_ISO_Left_Tab: e.key = '\t'; break;
                        case XK_Delete: case XK_BackSpace: e.key = keys::Delete; break;
                        case XK_Left: e.key = keys::Left; break;
                        case XK_Right: e.key = keys::Right; break;
                        case XK_Up: e.key = keys::Up; break;
                        case XK_Down: e.key = keys::Down; break;
                        case XK_F1: e.key = keys::F1; break;
                        default:
                            if (sym >= 0x20 && sym < 0x7f) e.key = static_cast<int>(sym);    // Latin-1 = ASCII
                            else continue;
                    }
                    return true;
                }
                case ButtonPress:
                case ButtonRelease: {
                    const unsigned b = ev.xbutton.button;
                    e.x = ev.xbutton.x;
                    e.y = ev.xbutton.y;
                    e.shift = (ev.xbutton.state & ShiftMask) != 0;
                    e.ctrl = (ev.xbutton.state & ControlMask) != 0;
                    if (b == Button4 || b == Button5) {         // X11 reports the wheel as buttons 4 and 5
                        if (ev.type == ButtonRelease) continue;
                        e.type = Event::Type::Wheel;
                        e.wheel = b == Button4 ? 1 : -1;
                        return true;
                    }
                    if (b > 3) continue;
                    e.type = ev.type == ButtonPress ? Event::Type::MouseDown : Event::Type::MouseUp;
                    e.button = static_cast<int>(b);
                    return true;
                }
                case MotionNotify:
                    e.type = Event::Type::MouseMove;
                    e.x = ev.xmotion.x;
                    e.y = ev.xmotion.y;
                    e.shift = (ev.xmotion.state & ShiftMask) != 0;
                    return true;
                default:
                    continue;
            }
        }
        return false;
    }

    void Window::present(const Framebuffer& fb) {
        Impl& m = *impl_;
        if (fb.width() != m.width || fb.height() != m.height) throw std::invalid_argument("Window::present: size mismatch");
        auto* out = reinterpret_cast<std::uint32_t*>(m.image->data);
        const int stride = m.image->bytes_per_line / 4;
        for (int y = 0; y < m.height; ++y)
            for (int x = 0; x < m.width; ++x) {
                const Color c = fb.getPixel(x, y);
                out[y * stride + x] = (std::uint32_t{c.r} << 16) | (std::uint32_t{c.g} << 8) | std::uint32_t{c.b};
            }
        XPutImage(m.display, m.window, m.gc, m.image, 0, 0, 0, 0, static_cast<unsigned>(m.width),
                  static_cast<unsigned>(m.height));
        XFlush(m.display);
    }

    void Window::setTitle(const std::string& title) {
        XStoreName(impl_->display, impl_->window, title.c_str());
    }
}
