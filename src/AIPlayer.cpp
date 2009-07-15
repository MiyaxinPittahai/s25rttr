// $Id: AIPlayer.h 4933 2009-05-24 12:29:23Z OLiver $
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

#include "main.h"
#include "AIPlayer.h"
#include "GameClientPlayer.h"
#include "GameWorld.h"
#include "GameCommands.h"

AIPlayer::AIPlayer(const unsigned char playerid, const GameWorldBase * const gwb, const GameClientPlayer * const player,
		const GameClientPlayerList * const players, const GlobalGameSettings * const ggs,
		const AI::Level level) : AIBase(playerid, gwb, player, players, ggs, level)
{
}


/// Wird jeden GF aufgerufen und die KI kann hier entsprechende Handlungen vollziehen
void AIPlayer::RunGF(const unsigned gf)
{
	// Testcode als Platzhalter:
	// Alle 500 gf eine Fahne setzen
	if(gf % 50 == 0)
	{
		int x = player->hqx - (rand()%16 -8);
		if(x < 0)
			x = 0;
		if(x >= gwb->GetWidth())
			x = gwb->GetWidth()-1;
		int y = player->hqy - (rand()%16 -8);
		if(y < 0)
			y = 0;
		if(y >= gwb->GetHeight())
			y = gwb->GetHeight()-1;

		gcs.push_back(new gc::SetFlag(x,y));

	}
}
