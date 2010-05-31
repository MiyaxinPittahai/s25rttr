// $Id: ctrlTab.h 6458 2010-05-31 11:38:51Z FloSoft $
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
#ifndef CTRLTAB_H_INCLUDED
#define CTRLTAB_H_INCLUDED

#pragma once

#include "Window.h"

#define MAX_TAB_COUNT 20

class ctrlGroup;

class ctrlTab : public Window
{
public:
	/// Konstruktor von @p ctrlTab.
	ctrlTab(Window *parent, unsigned int id, unsigned short x, unsigned short y, unsigned short width);

	/// f�gt eine Tab hinzu.
	ctrlGroup *AddTab(glArchivItem_Bitmap *image, std::string tooltip, const unsigned int id);
	/// l�scht alle Tabs.
	void DeleteAllTabs(void);
	/// aktiviert eine bestimmte Tabseite.
	void SetSelection(unsigned short nr, bool notify = false);
	/// Gibt ID des aktuell gew�hlten Tabs zur�ck
	unsigned int GetCurrentTab(void) const { return tabs[tab_selection]; }
	/// Gibt Tab-Group zur�ck, �ber die die Steuerelemente der Tab angesprochen werden k�nnen
	ctrlGroup *GetGroup(const unsigned int tab_id);
	/// Gibt aktuell ausgew�hlte Tab-Gruppe z�rck
	ctrlGroup *GetCurrentGroup() { return GetGroup(GetCurrentTab()); }

	virtual void Msg_Group_ButtonClick(const unsigned int group_id, const unsigned int ctrl_id);
	virtual void Msg_Group_EditEnter(const unsigned int group_id, const unsigned int ctrl_id);
	virtual void Msg_Group_EditChange(const unsigned int group_id, const unsigned int ctrl_id);
	virtual void Msg_Group_TabChange(const unsigned int group_id, const unsigned int ctrl_id, const unsigned short tab_id);
	virtual void Msg_Group_ListSelectItem(const unsigned int group_id, const unsigned int ctrl_id, const unsigned short selection);
	virtual void Msg_Group_ComboSelectItem(const unsigned int group_id, const unsigned int ctrl_id, const unsigned short selection);
	virtual void Msg_Group_CheckboxChange(const unsigned int group_id, const unsigned int ctrl_id, const bool checked);
	virtual void Msg_Group_ProgressChange(const unsigned int group_id, const unsigned int ctrl_id, const unsigned short position);
	virtual void Msg_Group_ScrollShow(const unsigned int group_id, const unsigned int ctrl_id, const bool visible);
	virtual void Msg_Group_OptionGroupChange(const unsigned int group_id, const unsigned int ctrl_id, const unsigned short selection);
	virtual void Msg_Group_Timer(const unsigned int group_id, const unsigned int ctrl_id);
	virtual void Msg_Group_TableSelectItem(const unsigned int group_id, const unsigned int ctrl_id, const unsigned short selection);
	virtual void Msg_Group_TableRightButton(const unsigned int group_id, const unsigned int ctrl_id, const unsigned short selection);
	virtual void Msg_Group_TableLeftButton(const unsigned int group_id, const unsigned int ctrl_id, const unsigned short selection);
	virtual void Msg_ButtonClick(const unsigned int ctrl_id);
	virtual bool Msg_LeftDown(const MouseCoords& mc);
	virtual bool Msg_LeftUp(const MouseCoords& mc);
	virtual bool Msg_WheelUp(const MouseCoords& mc);
	virtual bool Msg_WheelDown(const MouseCoords& mc);
	virtual bool Msg_MouseMove(const MouseCoords& mc);

protected:
	virtual bool Draw_(void);

private:
	unsigned short tab_count;
	unsigned short tab_selection;

	unsigned int tabs[MAX_TAB_COUNT];
};

#endif // !CTRLTAB_H_INCLUDED
