// Window backend for Windows: plain Win32 + GDI, no toolkit.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <windowsx.h>

#include "Window.h"

#include <deque>
#include <stdexcept>
#include <vector>

namespace ob::live {
    struct Window::Impl {
        HWND hwnd = nullptr;
        int width = 0, height = 0;
        std::deque<Event> queue;
        std::vector<std::uint32_t> pixels;      // 0x00RRGGBB, top-down
    };

    namespace {
        const wchar_t* kClassName = L"OptiBenchLiveWindow";

        bool shiftDown() { return (GetKeyState(VK_SHIFT) & 0x8000) != 0; }
        bool ctrlDown() { return (GetKeyState(VK_CONTROL) & 0x8000) != 0; }

        Event mouseEvent(Event::Type type, LPARAM lp, int button = 0) {
            Event e;
            e.type = type;
            e.button = button;
            e.x = GET_X_LPARAM(lp);
            e.y = GET_Y_LPARAM(lp);
            e.shift = shiftDown();
            e.ctrl = ctrlDown();
            return e;
        }

        LRESULT CALLBACK windowProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
            auto* m = reinterpret_cast<Window::Impl*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
            if (!m) return DefWindowProcW(hwnd, msg, wp, lp);
            switch (msg) {
                case WM_CLOSE: {
                    Event e;
                    e.type = Event::Type::Quit;
                    m->queue.push_back(e);
                    return 0;
                }
                case WM_PAINT: {
                    PAINTSTRUCT ps;
                    BeginPaint(hwnd, &ps);
                    EndPaint(hwnd, &ps);
                    Event e;
                    e.type = Event::Type::Redraw;
                    m->queue.push_back(e);
                    return 0;
                }
                case WM_KEYDOWN: {
                    Event e;
                    e.type = Event::Type::Key;
                    e.shift = shiftDown();
                    e.ctrl = ctrlDown();
                    switch (wp) {
                        case VK_ESCAPE: e.key = keys::Escape; break;
                        case VK_SPACE: e.key = keys::Space; break;
                        case VK_TAB: e.key = '\t'; break;
                        case VK_DELETE: case VK_BACK: e.key = keys::Delete; break;
                        case VK_LEFT: e.key = keys::Left; break;
                        case VK_RIGHT: e.key = keys::Right; break;
                        case VK_UP: e.key = keys::Up; break;
                        case VK_DOWN: e.key = keys::Down; break;
                        case VK_F1: e.key = keys::F1; break;
                        case VK_OEM_COMMA: e.key = ','; break;
                        case VK_OEM_PERIOD: e.key = '.'; break;
                        case VK_OEM_PLUS: e.key = '='; break;
                        case VK_OEM_MINUS: e.key = '-'; break;
                        default:
                            if (wp >= 'A' && wp <= 'Z') e.key = static_cast<int>(wp - 'A' + 'a');
                            else if (wp >= '0' && wp <= '9') e.key = static_cast<int>(wp);
                            else return 0;
                    }
                    m->queue.push_back(e);
                    return 0;
                }
                case WM_LBUTTONDOWN: SetCapture(hwnd); m->queue.push_back(mouseEvent(Event::Type::MouseDown, lp, 1)); return 0;
                case WM_MBUTTONDOWN: SetCapture(hwnd); m->queue.push_back(mouseEvent(Event::Type::MouseDown, lp, 2)); return 0;
                case WM_RBUTTONDOWN: SetCapture(hwnd); m->queue.push_back(mouseEvent(Event::Type::MouseDown, lp, 3)); return 0;
                case WM_LBUTTONUP: ReleaseCapture(); m->queue.push_back(mouseEvent(Event::Type::MouseUp, lp, 1)); return 0;
                case WM_MBUTTONUP: ReleaseCapture(); m->queue.push_back(mouseEvent(Event::Type::MouseUp, lp, 2)); return 0;
                case WM_RBUTTONUP: ReleaseCapture(); m->queue.push_back(mouseEvent(Event::Type::MouseUp, lp, 3)); return 0;
                case WM_MOUSEMOVE: m->queue.push_back(mouseEvent(Event::Type::MouseMove, lp)); return 0;
                case WM_MOUSEWHEEL: {
                    // wheel positions are in screen coordinates
                    POINT p{GET_X_LPARAM(lp), GET_Y_LPARAM(lp)};
                    ScreenToClient(hwnd, &p);
                    Event e;
                    e.type = Event::Type::Wheel;
                    e.wheel = GET_WHEEL_DELTA_WPARAM(wp) > 0 ? 1 : -1;
                    e.x = p.x;
                    e.y = p.y;
                    e.shift = shiftDown();
                    e.ctrl = ctrlDown();
                    m->queue.push_back(e);
                    return 0;
                }
                default:
                    return DefWindowProcW(hwnd, msg, wp, lp);
            }
        }

        std::wstring widen(const std::string& s) {
            if (s.empty()) return {};
            const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
            std::wstring w(static_cast<std::size_t>(n), L'\0');
            MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
            return w;
        }
    }

    Window::Window(const std::string& title, int width, int height) : impl_(std::make_unique<Impl>()) {
        Impl& m = *impl_;
        m.width = width;
        m.height = height;
        m.pixels.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);

        // a 1:1 image on high-DPI screens (otherwise Windows scales the window and blurs it)
        SetProcessDPIAware();

        const HINSTANCE instance = GetModuleHandleW(nullptr);
        WNDCLASSEXW wc{};
        wc.cbSize = sizeof wc;
        wc.lpfnWndProc = windowProc;
        wc.hInstance = instance;
        wc.hCursor = LoadCursorW(nullptr, reinterpret_cast<LPCWSTR>(IDC_ARROW));
        wc.lpszClassName = kClassName;
        RegisterClassExW(&wc);      // fails harmlessly if already registered

        const DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
        RECT r{0, 0, width, height};
        AdjustWindowRect(&r, style, FALSE);
        m.hwnd = CreateWindowExW(0, kClassName, widen(title).c_str(), style, CW_USEDEFAULT, CW_USEDEFAULT,
                                 r.right - r.left, r.bottom - r.top, nullptr, nullptr, instance, nullptr);
        if (!m.hwnd) throw std::runtime_error("CreateWindowEx failed");
        SetWindowLongPtrW(m.hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(&m));
        ShowWindow(m.hwnd, SW_SHOW);
        UpdateWindow(m.hwnd);
    }

    Window::~Window() {
        if (impl_->hwnd) {
            SetWindowLongPtrW(impl_->hwnd, GWLP_USERDATA, 0);
            DestroyWindow(impl_->hwnd);
        }
    }

    bool Window::poll(Event& e) {
        Impl& m = *impl_;
        MSG msg;
        while (m.queue.empty() && PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (m.queue.empty()) return false;
        e = m.queue.front();
        m.queue.pop_front();
        return true;
    }

    void Window::present(const Framebuffer& fb) {
        Impl& m = *impl_;
        if (fb.width() != m.width || fb.height() != m.height) throw std::invalid_argument("Window::present: size mismatch");
        for (int y = 0; y < m.height; ++y)
            for (int x = 0; x < m.width; ++x) {
                const Color c = fb.getPixel(x, y);
                m.pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(m.width) + static_cast<std::size_t>(x)] =
                    (std::uint32_t{c.r} << 16) | (std::uint32_t{c.g} << 8) | std::uint32_t{c.b};
            }
        BITMAPINFO bi{};
        bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bi.bmiHeader.biWidth = m.width;
        bi.bmiHeader.biHeight = -m.height;      // negative: rows top-down
        bi.bmiHeader.biPlanes = 1;
        bi.bmiHeader.biBitCount = 32;
        bi.bmiHeader.biCompression = BI_RGB;
        HDC dc = GetDC(m.hwnd);
        SetDIBitsToDevice(dc, 0, 0, static_cast<DWORD>(m.width), static_cast<DWORD>(m.height), 0, 0, 0,
                          static_cast<UINT>(m.height), m.pixels.data(), &bi, DIB_RGB_COLORS);
        ReleaseDC(m.hwnd, dc);
    }

    void Window::setTitle(const std::string& title) {
        SetWindowTextW(impl_->hwnd, widen(title).c_str());
    }
}
