// $Id: nofMiner.h 6458 2010-05-31 11:38:51Z FloSoft $
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

#ifndef NOF_MINER_H_
#define NOF_MINER_H_

#include "nofWorkman.h"

class nobUsualBuilding;

/// Klasse f�r den Schreiner
class nofMiner : public nofWorkman
{
	/// Zeichnet ihn beim Arbeiten
	void DrawWorking(int x, int y);
	/// Gibt die ID in JOBS.BOB zur�ck, wenn der Beruf Waren raustr�gt (bzw rein)
	unsigned short GetCarryID() const;
	/// Der Arbeiter erzeugt eine Ware
	GoodType ProduceWare();

public:

	nofMiner(const unsigned short x, const unsigned short y,const unsigned char player,nobUsual * workplace);
	nofMiner(SerializedGameData * sgd, const unsigned obj_id);

	GO_Type GetGOT() const { return GOT_NOF_MINER; }
};

#endif
