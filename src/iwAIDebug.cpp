// $Id: iwOptionsWindow.cpp 5853 2010-01-04 16:14:16Z FloSoft $
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
#include "iwAIDebug.h"

#include "WindowManager.h"
#include "controls.h"

#include "GameClient.h"
#include "GameServer.h"
#include "AIPlayerJH.h"
#include <sstream>
#include "GameWorld.h"

///////////////////////////////////////////////////////////////////////////////
// Makros / Defines
#if defined _WIN32 && defined _DEBUG && defined _MSC_VER
	#define new new(_NORMAL_BLOCK, THIS_FILE, __LINE__)
	#undef THIS_FILE
	static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////
/**
 *  Konstruktor von @p iwOptionsWindow.
 *
 *  @author jh
 */
	iwAIDebug::iwAIDebug(GameWorldViewer * const gwv)
	: IngameWindow(CGI_OPTIONSWINDOW, 0xFFFF, 0xFFFF, 300, 515, _("AI Debug"), LOADER.GetImageN("resource", 41)), gwv(gwv)
{
	// Nur Host hat Zugriff auf die Daten �ber die KI-Spieler
	if (!GameClient::inst().IsHost())
	{
		Close();
		return;
	}

	// Erstmal die AIs einsammeln
	for (unsigned i=0; i<GameClient::inst().GetPlayerCount(); ++i)
	{
		AIPlayerJH *ai = dynamic_cast<AIPlayerJH *>(GameServer::inst().GetAIPlayer(i));
		if (ai)
		{
			ais.push_back(ai);
		}
	}

	// Wenn keine KI-Spieler, schlie�en
	if (ais.size() == 0)
	{
		Close();
		return;
	}

	ctrlComboBox *players = AddComboBox(0, 15, 30, 250, 20, TC_GREY, NormalFont, 100);
	for (std::vector<AIPlayerJH *>::iterator it = ais.begin(); it != ais.end(); it++)
	{
		players->AddString(_((*it)->GetPlayer()->name));
	}
	selection = 0;
	players->SetSelection(selection);

	ctrlComboBox *overlays = AddComboBox(1, 15, 60, 250, 20, TC_GREY, NormalFont, 100);
	overlays->AddString("None");
	overlays->AddString("BuildingQuality");
	overlays->AddString("Reachability");
	overlays->AddString("Farmed");
	overlays->AddString("Wood");
	overlays->AddString("Stones");
	overlays->AddString("Gold");
	overlays->AddString("Ironore");
	overlays->AddString("Coal");
	overlays->AddString("Granite");
	overlays->AddString("Plantspace");
	overlays->AddString("Borderland");
	overlays->AddString("Fish");
	overlay = 0;
	overlays->SetSelection(overlay);

	//jobs = AddList(1, 15, 60, 120, 220, TC_GREY, NormalFont);

	text = AddText(2, 15, 120, "", COLOR_YELLOW, 
		glArchivItem_Font::DF_LEFT|glArchivItem_Font::DF_TOP, LOADER.GetFontN("resource", 0));

	//for(unsigned char i = 0; i < 31; ++i)
	//	list->AddString(_(BUILDING_NAMES[GAMECLIENT.visual_settings.build_order[i]]));
	//list->SetSelection(0);
}

void iwAIDebug::Msg_ComboSelectItem(const unsigned int ctrl_id, const unsigned short select)
{
	switch(ctrl_id)
	{
	default:
		break;

	case 0:
		{
			selection = select;
			gwv->SetAIDebug(overlay, ais[selection]->GetPlayerID(), true);
		} break;
	case 1:
		{
			overlay = select;
			gwv->SetAIDebug(overlay, ais[selection]->GetPlayerID(), true);
		} break;
	}
}

void iwAIDebug::Msg_PaintBefore()
{
	std::stringstream ss;

	AIJH::Job *currentJob = ais[selection]->GetCurrentJob();
	if (!currentJob) {
		text->SetText(_("No current job"));
		return;
	}

	AIJH::BuildJob *bj = dynamic_cast<AIJH::BuildJob *>(currentJob);
	AIJH::EventJob *ej = dynamic_cast<AIJH::EventJob *>(currentJob);

	if (bj)
	{
		ss << "BuildJob:" << std::endl;
		ss << BUILDING_NAMES[bj->type] << std::endl;
		ss << bj->target_x << " / " << bj->target_y << std::endl;
	}
	else if (ej)
	{
		ss << "EventJob:" << std::endl;
		switch(ej->ev->GetType())
		{
		case AIEvent::BuildingDestroyed: ss << "BuildingDestroyed" << std::endl; break;
		case AIEvent::BuildingConquered: ss << "BuildingConquered" << std::endl; break;
		case AIEvent::BuildingLost: ss << "BuildingLost" << std::endl; break;
		case AIEvent::BuildingOccupied: ss << "BuildingOccupied" << std::endl; break;
		case AIEvent::BorderChanged: ss << "BorderChanged" << std::endl; break;
		case AIEvent::TerritoryLost: ss << "TerritoryLost" << std::endl; break;
		case AIEvent::NoMoreResourcesReachable: ss << "NoMoreResourcesReachable" << std::endl; break;
		case AIEvent::BuildingFinished: ss << "BuildingFinished" << std::endl; break;
		case AIEvent::ExpeditionWaiting: ss << "ExpeditionWaiting" << std::endl; break;
		default: ss << "Unknown Event" << std::endl; break;
		}

		AIEvent::Building *evb = dynamic_cast<AIEvent::Building *>(ej->ev);
		if (evb)
		{
			ss << evb->GetX() << " / " << evb->GetY() << std::endl;
			ss << BUILDING_NAMES[evb->GetBuildingType()] << std::endl;
		}
	}

	switch(currentJob->status)
	{
	case AIJH::JOB_WAITING: ss << "JOB_WAITING"; break;
	case AIJH::JOB_EXECUTING_START: ss << "JOB_EXECUTING_START"; break;
	case AIJH::JOB_EXECUTING_ROAD1: ss << "JOB_EXECUTING_ROAD1"; break;
	case AIJH::JOB_EXECUTING_ROAD2: ss << "JOB_EXECUTING_ROAD2"; break;
	case AIJH::JOB_EXECUTING_ROAD2_2: ss << "JOB_EXECUTING_ROAD2_2"; break;
	case AIJH::JOB_FINISHED: ss << "JOB_FINISHED"; break;
	case AIJH::JOB_FAILED: ss << "JOB_FAILED"; break;
	default: ss << "Unknown status"; break;
	}

	text->SetText(ss.str());
}
