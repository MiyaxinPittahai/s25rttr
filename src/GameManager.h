// Copyright (c) 2005 - 2011 Settlers Freaks (sf-team at siedler25.org)
// Copyright (c) 2013 S25RTTR-Aux/Nevik Rehnel (hai.kataker at gmx.de)
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
#ifndef GAMEMANAGER_H_INCLUDED
#define GAMEMANAGER_H_INCLUDED

#include "Singleton.h"

/**
 * Various cursors and their indices in resource.idx
 */
enum CursorType {
    CURSOR_NONE       =  0,
    CURSOR_HAND       =  1,
    CURSOR_SCROLL     = 32,
    CURSOR_MOON       = 33,
    CURSOR_RM         = 34,
    CURSOR_RM_PRESSED = 35
};

/** 
 * "THE" GameManager class
 */
class GameManager : public Singleton<GameManager> {
public:
    GameManager(void);

    bool Start();
    void Stop();
    bool Run();

    bool StartMenu();
    bool ShowMenu();

    /**
     * reset average FPS counter
     */
    inline void ResetAverageFPS(void) {
        run_time = 0;
        frame_count = 0;
    }

    inline unsigned int GetRuntime(void) {
        return run_time;
    }

    inline unsigned int GetFrameCount(void) {
        return frame_count;
    }

    inline unsigned int GetAverageFPS(void) {
        if(run_time == 0)
            return 0;
        return (frame_count / run_time);
    }

    inline unsigned int GetFPS(void) {
        return framerate;
    }

    void SetCursor(CursorType cursor = CURSOR_HAND, bool once = false);

private:
    void DrawCursor();

private:
    unsigned int frames;
    unsigned int frame_count;
    unsigned int framerate;
    unsigned int frame_time;
    unsigned int run_time;
    unsigned int last_time;
    CursorType cursor;
    CursorType cursor_next;
};

#define GAMEMANAGER GameManager::inst()

#endif // GAMEMANAGER_H_INCLUDED
