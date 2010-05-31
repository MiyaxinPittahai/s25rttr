// $Id: dskIntro.cpp 6458 2010-05-31 11:38:51Z FloSoft $
//
// Copyright (c) 2005 - 2010 Settlers Freaks (sf-team at siedler25.org)
//
// This file is part of Siedler II.5 RTTR.
//
// Siedler II.5 RTTR is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
//
// Siedler II.5 RTTR is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Siedler II.5 RTTR. If not, see <http://www.gnu.org/licenses/>.

///////////////////////////////////////////////////////////////////////////////
// Header
#include "main.h"
#include "dskIntro.h"

#include "WindowManager.h"
#include "Loader.h"

#include "dskMainMenu.h"

///////////////////////////////////////////////////////////////////////////////
// Makros / Defines
#if defined _WIN32 && defined _DEBUG && defined _MSC_VER
	#define new new(_NORMAL_BLOCK, THIS_FILE, __LINE__)
	#undef THIS_FILE
	static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////
/** @class dskIntro
 *
 *  Klasse des Intro Desktops.
 *
 *  @author FloSoft
 */

///////////////////////////////////////////////////////////////////////////////
/**
 *  Konstruktor von @p dskIntro.
 *
 *  @author FloSoft
 */
dskIntro::dskIntro(void) : Desktop(LOADER.GetImageN("menu", 0))
{
	// "Zur�ck"
	AddTextButton(0, 300, 550, 200, 22, TC_RED1, _("Back"),NormalFont);

	AddImage(11, 20, 20, LOADER.GetImageN("logo", 0));
}

void dskIntro::Msg_ButtonClick(const unsigned int ctrl_id)
{
	switch(ctrl_id)
	{
	case 0: // "Zur�ck"
		{
			WindowManager::inst().Switch(new dskMainMenu);
		} break;
	}
}

