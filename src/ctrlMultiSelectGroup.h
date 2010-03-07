// $Id: ctrlMultiSelectGroup.h 5853 2010-01-04 16:14:16Z FloSoft $
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
#ifndef CTRLMULTISELECTGROUP_H_INCLUDED
#define CTRLMULTISELECTGROUP_H_INCLUDED

#pragma once

#include "ctrlGroup.h"
#include "ctrlButton.h"
#include <set>

/// Verwaltet eine Gruppe von n Buttons, von denen 0 bis n gleichzeitig ausgew�hlt sind
class ctrlMultiSelectGroup : public ctrlGroup
{
public:
	enum {
		ILLUMINATE = 0,
		CHECK,
		SHOW
	};

public:
	/// Konstruktor von @p ctrlMultiSelectGroup.
	ctrlMultiSelectGroup(Window *parent, unsigned int id, int select_type, bool scale = false);

	/// Selektiert einen neuen Button
	void AddSelection(unsigned short selection, bool notify = false);
	/// Entfernt einen selektierten Button aus der Selektion
	void RemoveSelection(unsigned short selection, bool notify = false);
	/// Wechselt zwischen selektiert/nicht selektiert
	void ToggleSelection(unsigned short selection, bool notify = false);
	/// Gibt Liste der aktuell selektierten Buttons zur�ck
	const std::set<unsigned short> &GetSelection() const { return selection; }
	/// Pr�ft ob ein Button ausgew�hlt ist
	bool IsSelected(unsigned short selection) const;
	// Gibt einen Button aus der Gruppe zur�ck zum direkten Bearbeiten
	ctrlButton *GetButton(unsigned int id) { return GetCtrl<ctrlButton>(id); }

	virtual void Msg_ButtonClick(const unsigned int ctrl_id);
	virtual bool Msg_LeftDown(const MouseCoords& mc);
	virtual bool Msg_LeftUp(const MouseCoords& mc);
	virtual bool Msg_WheelUp(const MouseCoords& mc);
	virtual bool Msg_WheelDown(const MouseCoords& mc);
	virtual bool Msg_MouseMove(const MouseCoords& mc);

protected:
	/// Zeichenmethode.
	virtual bool Draw_(void);

private:
	std::set<unsigned short> selection; ///< aktuell ausgew�hlte Buttons
	int select_type;         ///< Typ der Selektierung
};

#endif // !CTRLMULTISELECTGROUP_H_INCLUDED