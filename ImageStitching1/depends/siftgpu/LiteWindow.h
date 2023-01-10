#ifndef LITE_WINDOW_H
#define LITE_WINDOW_H

//#define WINDOW_PREFER_GLUT

#if defined(WINDOW_PREFER_GLUT)

#ifdef __APPLE__
#include "GLUT/glut.h"
#else
#include "GL/glut.h"
#endif
// for apple, use GLUT to create the window..
class LiteWindow {
    int glut_id;

public:
    LiteWindow() { glut_id = 0; }
    int IsValid() { return glut_id > 0; }
    virtual ~LiteWindow() {
        if (glut_id > 0)
            glutDestroyWindow(glut_id);
    }
    void MakeCurrent() { glutSetWindow(glut_id); }
    void Create(int x = -1, int y = -1, const char *display = NULL) {
        static int _glut_init_called = 0;
        if (glut_id != 0)
            return;

        // see if there is an existing window
        if (_glut_init_called)
            glut_id = glutGetWindow();

        // create one if no glut window exists
        if (glut_id != 0)
            return;

        if (_glut_init_called == 0) {
            int argc = 1;
            char *argv[4] = {"-iconic", 0, 0, 0};
            if (display) {
                argc = 3;
                argv[1] = "-display";
                argv[2] = (char *)display;
            }
            glutInit(&argc, argv);
            glutInitDisplayMode(GLUT_RGBA);
            _glut_init_called = 1;
        }
        if (x != -1)
            glutInitWindowPosition(x, y);
        if (display || x != -1)
            std::cout << "Using display [" << (display ? display : "\0") << "] at (" << x << "," << y << ")\n";
        glut_id = glutCreateWindow("SIFT_GPU_GLUT");
        glutHideWindow();
    }
};
#elif defined(_WIN32)

#ifndef _INC_WINDOWS
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

class LiteWindow {
    HWND hWnd;
    HGLRC hContext;
    HDC hdc;

public:
    LiteWindow() {
        hWnd = NULL;
        hContext = NULL;
        hdc = NULL;
    }
    virtual ~LiteWindow() {
        if (hContext)
            wglDeleteContext(hContext);
        if (hdc)
            ReleaseDC(hWnd, hdc);
        if (hWnd)
            DestroyWindow(hWnd);
    }
    int IsValid() { return hContext != NULL; }

    // display is ignored under Win32
    void Create(int x = -1, int y = -1, const char *display = NULL) {
        if (hContext)
            return;
        WNDCLASSEXA wcex = {sizeof(WNDCLASSEXA),
                            CS_HREDRAW | CS_VREDRAW,
                            (WNDPROC)DefWindowProcA,
                            0,
                            4,
                            0,
                            0,
                            0,
                            0,
                            0,
                            ("SIFT_GPU_LITE"),
                            0};
        RegisterClassExA(&wcex);
        hWnd = CreateWindowA("SIFT_GPU_LITE", "SIFT_GPU", 0, CW_USEDEFAULT, CW_USEDEFAULT, 100, 100, NULL, NULL, 0, 0);

        // move the window so that it can be on the second monitor
        if (x != -1) {
            MoveWindow(hWnd, x, y, 100, 100, 0);
            std::cout << "CreateWindow at (" << x << "," << y << ")\n";
        }

        ///////////////////////////////////////////////////
        PIXELFORMATDESCRIPTOR pfd = {sizeof(PIXELFORMATDESCRIPTOR),
                                     1,
                                     PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL,
                                     PFD_TYPE_RGBA,
                                     16,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     16,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0};
        hdc = GetDC(hWnd);
        ////////////////////////////////////
        int pixelformat = ChoosePixelFormat(hdc, &pfd);
        DescribePixelFormat(hdc, pixelformat, sizeof(pfd), &pfd);
        SetPixelFormat(hdc, pixelformat, &pfd);
        hContext = wglCreateContext(hdc);
    }
    void MakeCurrent() { wglMakeCurrent(hdc, hContext); }
};

#else

#include <GLFW/glfw3.h>
#include <cstdlib>
#include <iostream>

static void error_callback(int error, const char *description) {
    std::cerr << "Error: " << std::endl;
}
class LiteWindow {
public:
    LiteWindow() { window_ = nullptr; }
    virtual ~LiteWindow() {}
    int IsValid() { return window_ != nullptr; }
    void MakeCurrent() { glfwMakeContextCurrent(window_); }
    void Create(int x = -1, int y = -1, const char *display = NULL) {
        if (window_ != nullptr)
            return;

        glfwSetErrorCallback(error_callback);
        if (!glfwInit())
            std::abort();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

        window_ = glfwCreateWindow(1, 1, "Simple example", NULL, NULL);
        if (window_ == nullptr) {
            std::abort();
        }
    }

    GLFWwindow *window_;
};

#endif

#endif
