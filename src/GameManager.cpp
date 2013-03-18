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

///////////////////////////////////////////////////////////////////////////////
// Headers
#include "main.h"
#include "GameManager.h"

#include "GlobalVars.h"
#include "Settings.h"

#include "SoundManager.h"
#include "WindowManager.h"
#include "VideoDriverWrapper.h"
#include "AudioDriverWrapper.h"

#include "LobbyClient.h"
#include "GameServer.h"
#include "GameClient.h"

#include "dskSplash.h"
#include "dskMainMenu.h"
#include "dskLobby.h"
#include "iwMusicPlayer.h"

#include "MusicPlayer.h"

///////////////////////////////////////////////////////////////////////////////
// Macros / Defines
#if defined _WIN32 && defined _DEBUG && defined _MSC_VER
#   define new new(_NORMAL_BLOCK, THIS_FILE, __LINE__)
#   undef THIS_FILE
    static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////
/**
 *  Constructor for GameManager
 *
 *  @author OLiver
 *  @author Nevik Rehnel
 */
GameManager::GameManager(void) : 
        frames(0), 
        frame_count(0), 
        framerate(0), 
        frame_time(0), 
        run_time(0), 
        last_time(0), 
        cursor(CURSOR_HAND), 
        cursor_next(CURSOR_HAND)
{}

///////////////////////////////////////////////////////////////////////////////
/**
 *  Start game
 *
 *  @author FloSoft
 *  @author Nevik Rehnel
 */
bool GameManager::Start() {
    if (!SETTINGS.Load())
        return false;

    // load video driver
    if (!VideoDriverWrapper::inst().LoadDriver()) {
        LOG.lprintf("Video driver couldn't be loaded!\n"); //TODO: use gettext
        return false;
    }

    // for fullscreen mode: check if selected video mode exists
    if (SETTINGS.video.fullscreen) {
        std::vector<VideoDriver::VideoMode> available_video_modes;
        VideoDriverWrapper::inst().ListVideoModes(available_video_modes);

        bool found = false;
        for (size_t i = 0;i<available_video_modes.size();++i) {
            if (available_video_modes[i].width == SETTINGS.video.fullscreen_width &&
                    available_video_modes[i].height == SETTINGS.video.fullscreen_height)
                found = true;
        }

        if (!found && available_video_modes.size()) {
            // selected video mode was not found; use first available one
            SETTINGS.video.fullscreen_width = available_video_modes[0].width;
            SETTINGS.video.fullscreen_height = available_video_modes[0].height;
        }
    }

    { // create window
        unsigned short width, height;
        if (SETTINGS.video.fullscreen) {
            width = SETTINGS.video.fullscreen_width;
            height = SETTINGS.video.fullscreen_height;
        } else {
            width = SETTINGS.video.windowed_width;
            SETTINGS.video.windowed_height;
        }
        if (!VideoDriverWrapper::inst().CreateScreen(width, height, 
                SETTINGS.video.fullscreen))
            return false;
    }

    // load audio drivers
    if (!AudioDriverWrapper::inst().LoadDriver()) {
        LOG.lprintf("Audio driver couldn't be loaded!\n"); //TODO: use gettext
    }

    /// set the volume levels from settings
    AudioDriverWrapper::inst().SetMasterEffectVolume(SETTINGS.sound.effekte_volume);
    AudioDriverWrapper::inst().SetMasterMusicVolume(SETTINGS.sound.musik_volume);

    // save settings
    SETTINGS.Save();

    LOG.lprintf("\nStarte das Spiel\n"); //TODO: use gettext
    if (!StartMenu())
        return false;

    std::string playlist = iwMusicPlayer::GetFullPlaylistPath(SETTINGS.sound.playlist);
    if (MusicPlayer::inst().Load(playlist))
        MusicPlayer::inst().Play();

    return true;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * End game
 *
 * @author FloSoft
 * @author Nevik Rehnel
 */
void GameManager::Stop() {
    // save settings
    SETTINGS.Save();

    // exit window
    VideoDriverWrapper::inst().DestroyScreen();
}

///////////////////////////////////////////////////////////////////////////////
/**
 *  Main program loop
 *
 *  @author FloSoft
 *  @author Nevik Rehnel
 */
bool GameManager::Run() {
    // message "loop" (step)
    if (!VideoDriverWrapper::inst().Run())
        GLOBALVARS.notdone = false;

    LOBBYCLIENT.Run();

    GAMECLIENT.Run();
    GAMESERVER.Run();

    unsigned int current_time = VideoDriverWrapper::inst().GetTickCount();

    // SW VSync (with 4% tolerance)
    if (SETTINGS.video.vsync > 1) {
        static unsigned long vsync = SETTINGS.video.vsync;

        // add or remove 10% until we reach the target framerate
        if (vsync < 200 && 1000 * framerate < (unsigned int)(960 * vsync) )
            vsync = (1100 * vsync) / 1000;
        else if (vsync > SETTINGS.video.vsync)
            vsync = (900 * vsync) / 1000;
        else
            vsync = SETTINGS.video.vsync;

        unsigned long goal_ticks = 960*1000*1000 / vsync; //target step length
#ifdef _WIN32
        if(goal_ticks < 13 * 1000 * 1000) // timer resolutions <13ms do not work correctly for Windows
            goal_ticks = 0;
#endif // !_WIN32

        // if we do have a target step length and the last step was faster than
        // the target step length: wait a bit
        if (goal_ticks > 0 && (current_time - last_time)*1000*1000 < goal_ticks &&
                (current_time >= last_time)) {
            //figure out how long to wait
            struct timespec req;
            req.tv_sec  = 0;
            req.tv_nsec = goal_ticks - (current_time - last_time)*1000*1000 ;

            //and wait
            while (nanosleep(&req, &req) == -1)
                continue;  

            current_time = VideoDriverWrapper::inst().GetTickCount();
        }
    }

    WindowManager::inst().Draw();

    last_time = current_time;

    if ((GAMECLIENT.GetState() == GameClient::CS_GAME) && (GAMECLIENT.GetGFLength() < 30)) {
        LOADER.GetImageN("io", 164)->Draw( //TODO: figure out what this does
                VideoDriverWrapper::inst().GetScreenWidth() - 55, //dst_x
                35, //dst_y
                0, //dst_w
                0, //dst_h
                0, //src_x
                0); //src_y
    }

    DrawCursor();

    // calculate framerate
    if (current_time - frame_time >= 1000) {
        // a whole second has passed
        ++run_time;

        // increase the number of drawn frames
        frame_count += frames;

        // calculate normal framerate
        framerate = frames;
        frames = 0;

        frame_time = current_time;
    }

    // and print it on screen
    char frame_str[64];
    sprintf(frame_str, "%d fps", framerate);

    SmallFont->Draw( VideoDriverWrapper::inst().GetScreenWidth(), 0, frame_str, glArchivItem_Font::DF_RIGHT, COLOR_YELLOW);

    // swap drawing buffer
    VideoDriverWrapper::inst().SwapBuffers();

    ++frames;

    // clean up window manager
    if (GLOBALVARS.notdone == false)
        WindowManager::inst().CleanUp();

    return GLOBALVARS.notdone;
}

///////////////////////////////////////////////////////////////////////////////
/**
 *  Starts and loads the menu
 *
 *  @author FloSoft
 *  @author Nevik Rehnel
 */
bool GameManager::StartMenu() {
    // load general files
    if (!LOADER.LoadFilesAtStart()) {
        error("Einige Dateien konnten nicht geladen werden.\n" //use gettext
            "Stellen Sie sicher, dass die Siedler 2 Gold-Edition im gleichen \n"
            "Verzeichnis wie Return to the Roots installiert ist.");

        error("Some files failed to load.\n"
            "Please ensure that the Settlers 2 Gold-Edition is installed \n"
            "in the same directory as Return to the Roots.");

        return false;
    }

    // show splash screen
    WindowManager::inst().Switch(new dskSplash());

    return true;
}

///////////////////////////////////////////////////////////////////////////////
/**
 *  Show the main menu
 *
 *  @author FloSoft
 *  @author Nevik Rehnel
 */
bool GameManager::ShowMenu()
{
    GAMECLIENT.Stop();
    GAMESERVER.Stop();
    SOUNDMANAGER.StopAll();

    GameClient::inst().SetInterface(NULL);

    // we are no longer ingame
    GLOBALVARS.ingame = false;

    // if we are logged into the lobby
    if (LOBBYCLIENT.LoggedIn())
        // show lobby
        WindowManager::inst().Switch(new dskLobby());
    else
        // actually show the main menu
        WindowManager::inst().Switch(new dskMainMenu());

    return true;
}

///////////////////////////////////////////////////////////////////////////////
/**
 *  Set the cursor type
 *
 *  @author Divan
 */
void GameManager::SetCursor(CursorType cursor, bool once) {
	cursor_next = cursor;
	if(!once)
        this->cursor = cursor;
	return;
}

///////////////////////////////////////////////////////////////////////////////
/**
 *  Draw the cursor
 *
 *  @author Divan
 *  @author Nevik Rehnel
 */
void GameManager::DrawCursor() {
    // draw mouse cursor
    switch (cursor_next) {
        case CURSOR_HAND:
            if (VideoDriverWrapper::inst().IsLeftDown())
                LOADER.GetImageN("resource", 31)->Draw(
                        VideoDriverWrapper::inst().GetMouseX(),
                        VideoDriverWrapper::inst().GetMouseY(), 0, 0, 0, 0, 0, 0);
            else
                LOADER.GetImageN("resource", 30)->Draw(
                        VideoDriverWrapper::inst().GetMouseX(),
                        VideoDriverWrapper::inst().GetMouseY(), 0, 0, 0, 0, 0, 0);
            break;
        case CURSOR_SCROLL:
        case CURSOR_MOON:
        case CURSOR_RM:
        case CURSOR_RM_PRESSED:
            LOADER.GetImageN("resource", cursor_next)->Draw(
                    VideoDriverWrapper::inst().GetMouseX(),
                    VideoDriverWrapper::inst().GetMouseY(), 0, 0, 0, 0, 0, 0);
            break;
        case CURSOR_NONE:
        default: {}
    }

    cursor_next = cursor;
    return;
}
