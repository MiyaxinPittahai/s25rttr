// Copyright (c) 2013 S25RTTR-Aux/Nevik Rehnel (hai.kataker at gmx.de)
// Copyright (c) 2005 - 2011 Settlers Freaks (sf-team at siedler25.org)
//
// This file is part of Return To The Roots.
//
// Return To The Roots is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
//
// Return To The Roots is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Return To The Roots. If not, see <http://www.gnu.org/licenses/>.
#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

///////////////////////////////////////////////////////////////////////////////
// System headers

#define _CRTDBG_MAP_ALLOC

#ifdef _WIN32
#   define WIN32_LEAN_AND_MEAN
#   include <windows.h>
#   include <ws2tcpip.h>
#   include <shlwapi.h>
#   ifndef __CYGWIN__
#       include <conio.h>
#   endif //__CYGWIN__

#   ifdef _MSC_VER
#       include <crtdbg.h>
#   else
#       include <assert.h>
#   endif //_MSC_VER

#   undef PlaySound

#   include "win32_nanosleep.h"
#else // !_WIN32
#   include <unistd.h>
#   include <stdarg.h>
#   include <signal.h>
#   include <dirent.h>
#   include <dlfcn.h>
#   include <netdb.h>
#   include <netinet/in.h>
#   include <netinet/tcp.h>
#   include <sys/types.h>
#   include <sys/socket.h>
#   include <sys/select.h>
#   include <sys/ioctl.h>
#   include <sys/stat.h>
#   include <arpa/inet.h>
#   include <assert.h>

#   include "strlwr.h"
#endif // !_WIN32

#if defined _WIN32 && defined _DEBUG
#   include <crtdbg.h>
#endif // _WIN32 && _DEBUG

#include <errno.h>
#include <cstdlib>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <cmath>

#include <map>
#include <algorithm>
#include <vector>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include <bzlib.h>

extern "C" {
#   include "lua.h"
#   include "lualib.h"
#   include "lauxlib.h"
}

#ifdef __APPLE__
#   include <OpenGL/gl.h>
#   include <OpenGL/glext.h>
#else // !__APPLE__
#   include <GL/gl.h>
#   include <GL/glext.h>
#endif // !__APPLE__

#ifdef _WIN32
#   ifdef _MSC_VER
#       define getch _getch
#       ifndef snprintf
#           define snprintf _snprintf
#       endif // !snprintf
#       ifndef assert
#           define assert _ASSERT
#       endif // !assert
#   endif // _MSC_VER

    typedef int socklen_t;
#   define BREAKPOINT ;
#else // !_WIN32
#   define BREAKPOINT raise(SIGTRAP)
#   define SOCKET int
#   define INVALID_SOCKET -1
#   define SOCKET_ERROR -1
#   define HINSTANCE void*

#   define closesocket close
#   define LoadLibrary(x) dlopen(x, RTLD_LAZY)
#   define LoadLibraryW LoadLibrary
#   define LoadLibraryA LoadLibrary
#   define GetProcAddress(x, y) dlsym(x, y)
#   define GetProcAddressW GetProcAddress
#   define GetProcAddressA GetProcAddress
#   define FreeLibrary(x) dlclose(x)
#endif // !_WIN32

///////////////////////////////////////////////////////////////////////////////
// Eigene Header
#include <build_paths.h>
#include "../libutil/src/libutil.h"
#include "../mygettext/src/mygettext.h"
#include "../liblobby/src/liblobby.h"
#include "../libsiedler2/src/libsiedler2.h"
#include "../libendian/src/libendian.h"

#include "macros.h"
#include "list.h"
#include "Swap.h"

#include "glAllocator.h"
#include "../driver/src/Sound.h"

typedef struct Rect {
    unsigned short left, top, right, bottom;
} Rect;

///////////////////////////////////////////////////////////////////////////////
/**
 * Convert a void pointer to a function pointer, using a union. GCC will otherwise complain about
 * a "type punned pointer" or "ISO C++ forbids conversion"
 *
 * @author FloSoft
 */
template <typename F>
inline F pto2ptf(void *o) {
    union {
        F f;
        void *o;
    } U;
    U.o = o;

    return U.f;
}

#undef min
/** Determine and return the minimum of two objects using the right `<` operator for the type. */
template <typename T>
inline T min(T a, T b) { return ((a) < (b)) ? (a) : (b); }

#undef max
/** Determine and return the maximum of two objects using the right `<` operator for the type. */
template <typename T>
inline T max(T a, T b) { return ((a) < (b)) ? (b) : (a); }

/** Calculates the (absolute) difference of two (unsigned) values. Uses th `>` and `-` operators */
template <typename T>
inline T SafeDiff(T a, T b) { return ((a) > (b)) ? (a-b) : (b-a); }

/**
 * 2D Point
 */
template <typename T>
struct Point {
    T x,y;
    Point() {}
    Point(const T x, const T y) : x(x), y(y) {
    }
    bool operator==(const Point<T> second) const {
        return (x == second.x && y == second.y);
    }
};

const char *GetWindowTitle();       //in build_version.cpp, depending on build_version.h
const char *GetWindowVersion();     //in build_version.cpp, depending on build_version.h
const char *GetWindowRevision();    //in build_version.cpp, depending on build_version.h

const unsigned oo = 0xffffffff;     //==MAX_UINT32

#endif // MAIN_H_INCLUDED
