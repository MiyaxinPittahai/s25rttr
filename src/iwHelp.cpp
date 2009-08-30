// $Id: iwHelp.cpp 5464 2009-08-30 16:05:54Z FloSoft $
//
// Copyright (c) 2005-2009 Settlers Freaks (sf-team at siedler25.org)
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
#include "iwHelp.h"

#include "Loader.h"
#include "controls.h"
#include "VideoDriverWrapper.h"
#include "files.h"
#include "Settings.h"
#include "iwMsgbox.h"
#include "WindowManager.h"

///////////////////////////////////////////////////////////////////////////////
// Makros / Defines
#if defined _WIN32 && defined _DEBUG && defined _MSC_VER
	#define new new(_NORMAL_BLOCK, THIS_FILE, __LINE__)
	#undef THIS_FILE
	static char THIS_FILE[] = __FILE__;
#endif

/// Breite des Fensters
const unsigned short HELP_WINDOW_WIDTH = 240;

/// Maximale Anzahl von Zeilen, bis Scrollbar eingesetzt wird
const unsigned MAX_LINES = 15;

///////////////////////////////////////////////////////////////////////////////
/**
 *  Konstruktor von @p iwHelp.

 *  @author OLiver
 */

iwHelp::iwHelp(const GUI_ID gui_id, const std::string& title, const std::string& content)
	: IngameWindow(gui_id, 0xFFFE, 0xFFFE, HELP_WINDOW_WIDTH, 480, title, LOADER.GetImageN("resource", 41))
{
	glArchivItem_Font::WrapInfo wi;
	NormalFont->GetWrapInfo(content,HELP_WINDOW_WIDTH-28,HELP_WINDOW_WIDTH-28,wi);

	// Mehr Linien ben�tigt als die maximalen? Dann kommt ja noch die Scrollbar dran und der ganze Spa� muss
	// umgebrochen werden, also nochmal mit geringerer Breite berechnen
	if(wi.positions.size() > MAX_LINES)
		NormalFont->GetWrapInfo(content,HELP_WINDOW_WIDTH-28-ctrlMultiline::SCROLLBAR_WIDTH,
		HELP_WINDOW_WIDTH-24-ctrlMultiline::SCROLLBAR_WIDTH,wi);

	unsigned show_lines = min(wi.positions.size(),MAX_LINES);

	unsigned short text_height=show_lines*NormalFont->getHeight();

	// H�he setzen
	SetHeight(text_height+40);
	// Fenster neben die Maus schieben
	MoveNextToMouse();

	// Gr��e des Fensters und des Controls nach der Anzahl der Zeilen
	ctrlMultiline *text = AddMultiline(2, 10, 20, HELP_WINDOW_WIDTH-20, text_height+4, TC_GREEN1, NormalFont, glArchivItem_Font::DF_LEFT | glArchivItem_Font::DF_TOP);
	text->EnableBox(false);

	std::string * lines = new std::string[wi.positions.size()];
	wi.CreateSingleStrings(content,lines);
	for(unsigned i = 0;i<wi.positions.size();++i)
		text->AddString(lines[i],COLOR_YELLOW,false);


	delete [] lines;
}
