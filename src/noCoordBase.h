// $Id: noCoordBase.h 6458 2010-05-31 11:38:51Z FloSoft $
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
#ifndef NOCOORDBASE_H_INCLUDED
#define NOCOORDBASE_H_INCLUDED

#pragma once

#include "noBase.h"

class noCoordBase : public noBase
{

public:

	/// Konstruktor von @p noCoordBase.
	noCoordBase(const NodalObjectType nop, const unsigned short x, const unsigned short y) : noBase(nop), x(x), y(y) {}
	noCoordBase(SerializedGameData * sgd, const unsigned obj_id);

	/// Aufr�ummethoden
protected:	void Destroy_noCoordBase(void) { Destroy_noBase(); }
public:		void Destroy(void) { Destroy_noCoordBase(); }

	/// Serialisierungsfunktionen
	protected:	void Serialize_noCoordBase(SerializedGameData * sgd) const;
	public:		void Serialize(SerializedGameData *sgd) const { Serialize_noCoordBase(sgd); }

	/// liefert die X-Koordinate.
	unsigned short GetX(void) const { return x; }
	/// liefert die Y-Koordinate.
	unsigned short GetY(void) const { return y; }

	/// Liefert GUI-ID zur�ck f�r die Fenster
	unsigned CreateGUIID() const;

protected:
	unsigned short x; ///< X-Koordinate
	unsigned short y; ///< Y-Koordinate
};

#endif // !NOCOORDBASE_H_INCLUDED
