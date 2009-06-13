// $Id: iwPostWindow.cpp 5047 2009-06-13 20:32:24Z OLiver $
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
#include "iwPostWindow.h"
#include "ctrlText.h"
#include "ctrlButton.h"
#include "ctrlTable.h"
#include "ctrlImage.h"
#include "WindowManager.h"
#include "Loader.h"
#include "const_gui_ids.h"
#include "macros.h"
#include "GameClient.h"
#include "GameClientPlayer.h"
#include "GameCommands.h"
#include <iostream>
#include <sstream>
///////////////////////////////////////////////////////////////////////////////
// Makros / Defines
#if defined _WIN32 && defined _DEBUG && defined _MSC_VER
	#define new new(_NORMAL_BLOCK, THIS_FILE, __LINE__)
	#undef THIS_FILE
	static char THIS_FILE[] = __FILE__;
#endif

iwPostWindow::iwPostWindow(GameWorldViewer& gwv) 
: IngameWindow(CGI_POSTOFFICE, 0xFFFF, 0xFFFF, 254, 295, _("Post office"), GetImage(resource_dat, 41)), gwv(gwv)
{
	AddImageButton( 0, 18, 25, 35, 35,TC_GREY,GetImage(io_dat, 190));		// Viewer: 191 - Papier
	AddImageButton( 1, 56, 25, 35, 35,TC_GREY,GetImage(io_dat, 30));		// Viewer:  31 - Soldat
	AddImageButton( 2, 91, 25, 35, 35,TC_GREY,GetImage(io_dat, 20));		// Viewer:  21 - Geologe
	AddImageButton( 3,126, 25, 35, 35,TC_GREY,GetImage(io_dat, 28));		// Viewer:  29 - Wage
	AddImageButton( 4,161, 25, 35, 35,TC_GREY,GetImage(io_dat, 189));		// Viewer: 190 - Neue Nachricht
	AddImageButton( 5,199, 25, 35, 35,TC_GREY,GetImage(io_dat, 79));		// Viewer:  80 - Notiz
	AddImage(  6,126,151, GetImage(io_dat, 228));
	AddImageButton( 7, 18,242, 30, 35,TC_GREY,GetImage(io_dat, 225));		// Viewer: 226 - Hilfe
	AddImageButton( 8, 51,246, 30, 26,TC_GREY,GetImage(io_dat, 102));		// Viewer: 103 - Schnell zur�ck
	AddImageButton( 9, 81,246, 30, 26,TC_GREY,GetImage(io_dat, 103));		// Viewer: 104 - Zur�ck
	AddImageButton(10,111,246, 30, 26,TC_GREY,GetImage(io_dat, 104));		// Viewer: 105 - Vor
	AddImageButton(11,141,246, 30, 26,TC_GREY,GetImage(io_dat, 105));		// Viewer: 106 - Schnell vor


  gotoButton = AddImageButton(14,181,246, 30, 26,TC_GREY,GetImage(io_dat, 107));	// Goto, nur sichtbar wenn Nachricht mit Koordinaten da
  gotoButton->SetVisible(false);
  deleteButton = AddImageButton(15,211,246, 30, 26,TC_GREY,GetImage(io_dat, 106));	// M�lleimer, nur sichtbar, wenn Nachricht da
  deleteButton->SetVisible(false);

 

  postMsgInfos = AddText(18, 127, 228, "", MakeColor(255, 188, 100, 88), glArchivItem_Font::DF_CENTER | glArchivItem_Font::DF_BOTTOM, SmallFont);
  postMsgInfos->SetVisible(false);

  postImage = AddImage(13, 127, 155, GetImage(io_dat, 225));

  postText = AddText(12,126,151, _("No letters!"), 0xFF886034, glArchivItem_Font::DF_CENTER | glArchivItem_Font::DF_BOTTOM, GetFont(resource_dat, 0));

   acceptButton = AddImageButton(16,87,185, 30, 26,TC_GREEN1,GetImage(io_dat, 32));  // Button mit Haken, zum Annehmen von Vertr�gen
  acceptButton->SetVisible(false);
  declineButton = AddImageButton(17,137,185, 30, 26,TC_RED1,GetImage(io_dat, 40));  // Button mit Kreuz, zum Ablehnen von Vertr�gen
  declineButton->SetVisible(false);

  currentMessage = 0;
  DisplayPostMessage();
}

void iwPostWindow::Msg_ButtonClick(const unsigned int ctrl_id)
{
  switch(ctrl_id)
  {
    // Schnell zur�ck
  case 8: currentMessage = 0;
    DisplayPostMessage();
    break;
    // Zur�ck
  case 9: currentMessage = (currentMessage > 0) ? currentMessage - 1 : 0;
    DisplayPostMessage();
    break;
    // Vor
  case 10: currentMessage = (currentMessage < GameClient::inst().GetPostMessages().size()-1) ? currentMessage + 1 : GameClient::inst().GetPostMessages().size()-1;
    DisplayPostMessage();
    break;
    // Schnell vor
  case 11: currentMessage = GameClient::inst().GetPostMessages().size() - 1;
    DisplayPostMessage();
    break;

    // Goto
  case 14:
    {
      PostMsg *pm = GetPostMsg(currentMessage);
      PostMsgWithLocation *pml = dynamic_cast<PostMsgWithLocation*>(pm);
      if (pml)
      {
        gwv.MoveToMapObject(pml->GetX(), pml->GetY());
      }
    }
    break;

    // L�schen
  case 15:
    {
    PostMsg *pm = GetPostMsg(currentMessage);
    GameClient::inst().DeletePostMessage(pm);
    currentMessage = (currentMessage > 0) ? currentMessage - 1 : 0;
    }
    break;

    // Vertrag annehmen
	case 16:
	{
		PostMsg *pm = GetPostMsg(currentMessage);
		DiplomacyPostMsg *dpm = dynamic_cast<DiplomacyPostMsg*>(pm);
		if (dpm)
		{
			GameClient::inst().AddGC(new gc::AcceptPact(true,dpm->id,dpm->pt,dpm->player));

			GameClient::inst().DeletePostMessage(pm);
			currentMessage = (currentMessage > 0) ? currentMessage - 1 : 0;
		}
	} break;

    // Vertrag ablehnen
  case 17:
    {
      PostMsg *pm = GetPostMsg(currentMessage);
      DiplomacyPostMsg *dpm = dynamic_cast<DiplomacyPostMsg*>(pm);
      if (dpm)
      {
        //dpm->GetPlayerID();
        // TODO Damit was tun
        // GAMECLIENT.GetPlayer(dpm->GetPlayerID();)->MakeMeNotAlly(me);
      }
    }
    break;
  }
  
}

void iwPostWindow::Msg_PaintBefore()
{
  // Immer wenn sich die Anzahl der Nachrichten ge�ndert hat neu pr�fen was so angezeigt werden muss
  unsigned currentSize = GameClient::inst().GetPostMessages().size();
  if (currentSize != lastSize)
  {
    // Neue Nachrichten dazugekommen, w�hrend das Fenster offen ist: 
    // Ansicht der vorherigen Nachricht beibehalten, au�er es gab vorher gar keine Nachricht
    if (lastSize < currentSize && lastSize != 0)
    {
      currentMessage += currentSize - lastSize;

      // Falls das zuf�llig grad die 20 Nachrichtengrenze �berschritten hat: Auf letzte Nachricht springen
      if (currentMessage >= MAX_POST_MESSAGES)
        currentMessage = MAX_POST_MESSAGES - 1;
    }
    lastSize = GameClient::inst().GetPostMessages().size();

    // Anzeigeeinstellungen setzen
    DisplayPostMessage();
  }
}


// Holt Nachricht Nummer pos
PostMsg *iwPostWindow::GetPostMsg(unsigned pos) const
{
  PostMsg* pm = 0;
  unsigned counter = 0;
  for(std::list<PostMsg*>::const_iterator it = GameClient::inst().GetPostMessages().begin(); it != GameClient::inst().GetPostMessages().end(); ++it)
  {
    if (counter == pos)
    {
      pm = *it;
      break;
    }
    counter++;
  }
  assert(pm);
  return pm;
}

// Zeigt Nachricht an, passt Steuerelemente an
void iwPostWindow::DisplayPostMessage()
{
  const unsigned xImgBottomCenter = 127;
  const unsigned yImgBottomCenter = 210;

  // todo: koordinaten abschmecken
  const unsigned xTextTopCenter = 127;
  const unsigned yTextTopCenter = 110;
  
  const unsigned xTextCenter = 126;
  const unsigned yTextCenter = 151;

  unsigned size = GameClient::inst().GetPostMessages().size();

  // Keine Nachrichten, alles ausblenden, bis auf zentrierten Text
  if (size == 0)
  {
    postText->SetText(_("No letters!"));
    postText->Move(xTextCenter, yTextCenter);

    postImage->SetVisible(false);
    deleteButton->SetVisible(false);
    gotoButton->SetVisible(false);
    acceptButton->SetVisible(false);
    declineButton->SetVisible(false);
    postMsgInfos->SetVisible(false);
    return;
  }

  // Falls currentMessage au�erhalb der aktuellen Nachrichtenmenge liegt: korrigieren
  if (currentMessage >= size)
    currentMessage = size - 1;

  PostMsg *pm = GetPostMsg(currentMessage);

  // Nachrichten vorhanden, dann geht auf jeden Fall L�schbutton einblenden
  deleteButton->SetVisible(true);

  // Und ne Info-Zeile haben wir auch;
  std::stringstream ss;
  ss << _("Message") << " " << currentMessage+1 << " " << _("of") << " " << size << " - GF: " << pm->GetSendFrame();
  postMsgInfos->SetText(ss.str());
  postMsgInfos->SetVisible(true);



  // Rest abh�ngig vom Nachrichten-Typ
  switch (pm->GetType())
  {
  case PMT_IMAGE_WITH_LOCATION:
    {
    ImagePostMsgWithLocation *ipm = dynamic_cast<ImagePostMsgWithLocation*>(pm);
    assert(ipm);
    postText->SetText(pm->GetText());
    postText->Move(xTextTopCenter, yTextTopCenter);
    postImage->SetImage(ipm->GetImage_());
    postImage->Move(xImgBottomCenter + ipm->GetImage_()->getNx() - ipm->GetImage_()->getWidth()/2, 
      yImgBottomCenter + ipm->GetImage_()->getNy() - ipm->GetImage_()->getHeight());
    postImage->SetVisible(true);
    gotoButton->SetVisible(true);
    acceptButton->SetVisible(false);
    declineButton->SetVisible(false);
    }
    break;
  case PMT_WITH_LOCATION:
    {
    PostMsgWithLocation *pml = dynamic_cast<PostMsgWithLocation*>(pm);
    assert(pml);
    postText->SetText(pm->GetText());
    postText->Move(xTextCenter, yTextCenter);
    postImage->SetVisible(false);
    gotoButton->SetVisible(true);
    acceptButton->SetVisible(false);
    declineButton->SetVisible(false);
    }
    break;
  case PMT_DIPLOMACY:
    {
    DiplomacyPostMsg *dpm = dynamic_cast<DiplomacyPostMsg*>(pm);
    assert(dpm);
    postText->SetText(pm->GetText());
    postText->Move(xTextTopCenter, yTextTopCenter);
    //unsigned player = dpm->GetPlayerID();
    // Zwei Buttons einblenden...
    acceptButton->SetVisible(true);
    declineButton->SetVisible(true);
    gotoButton->SetVisible(false);
    postImage->SetVisible(false);
    }
    break;
  case PMT_NORMAL:
    postText->SetText(pm->GetText());
    postText->Move(xTextCenter, yTextCenter);
    postImage->SetVisible(false);
    gotoButton->SetVisible(false);
    acceptButton->SetVisible(false);
    declineButton->SetVisible(false);
    break;
  }
}
