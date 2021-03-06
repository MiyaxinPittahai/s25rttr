// $Id: nofDefender.cpp 7521 2011-09-08 20:45:55Z FloSoft $
//
// Copyright (c) 2005 - 2011 Settlers Freaks (sf-team at siedler25.org)
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
// Header
#include "main.h"
#include "nofDefender.h"

#include "nofAttacker.h"
#include "nobMilitary.h"
#include "Loader.h"
#include "GameClient.h"
#include "GameConsts.h"
#include "Random.h"
#include "GameWorld.h"
#include "noFighting.h"
#include "nofPassiveSoldier.h"
#include "SerializedGameData.h"

///////////////////////////////////////////////////////////////////////////////
// Makros / Defines
#if defined _WIN32 && defined _DEBUG && defined _MSC_VER
	#define new new(_NORMAL_BLOCK, THIS_FILE, __LINE__)
	#undef THIS_FILE
	static char THIS_FILE[] = __FILE__;
#endif


nofDefender::nofDefender(const unsigned short x, const unsigned short y,const unsigned char player,
											 nobBaseMilitary * const home,const unsigned char rank, nofAttacker * const attacker)
											 : nofActiveSoldier(x,y,player,home,rank,STATE_DEFENDING_WALKINGTO), attacker(attacker)
{
}

nofDefender::nofDefender(nofPassiveSoldier * other, nofAttacker * const attacker)
: nofActiveSoldier(*other,STATE_DEFENDING_WALKINGTO), attacker(attacker)
{
}

void nofDefender::Serialize_nofDefender(SerializedGameData * sgd) const
{
	Serialize_nofActiveSoldier(sgd);

	if(state != STATE_FIGUREWORK)
		sgd->PushObject(attacker,true);
}

nofDefender::nofDefender(SerializedGameData * sgd, const unsigned obj_id) :nofActiveSoldier(sgd,obj_id)
{
	if(state != STATE_FIGUREWORK)
		attacker = sgd->PopObject<nofAttacker>(GOT_NOF_ATTACKER);
	else
		attacker = 0;
}

/// wenn man gelaufen ist
void nofDefender::Walked()
{
	// Was bestimmtes machen, je nachdem welchen Status wir gerade haben
	switch(state)
	{
	case STATE_DEFENDING_WALKINGTO:
		{
			// Mit Angreifer den Kampf beginnen
			gwg->AddFigure(new noFighting(attacker,this),x,y);
			state = STATE_FIGHTING;
			attacker->state = STATE_ATTACKING_FIGHTINGVSDEFENDER;

		} break;
	case STATE_DEFENDING_WALKINGFROM:
		{
			// Ist evtl. unser Heimatgeb�ude zerst�rt?
			if(!building)
			{
				// Rumirren
				attacker = 0;
				state = STATE_FIGUREWORK;
				StartWandering();
				Wander();

				return;
			}

			// Zu Hause angekommen

			// Ist evtl. wieder ein Angreifer in der Zwischenzeit an der Fahne angekommen?
			if(attacker)
			{
				// dann umdrehen und wieder rausgehen
				state = STATE_DEFENDING_WALKINGTO;
				StartWalking(4);
			}
			else
			{
				// mich von der Landkarte tilgen
				gwg->RemoveFigure(this,x,y);
				// mich zum Geb�ude wieder hinzuf�gen
				building->AddActiveSoldier(this);
				// Geb�ude Bescheid sagen, dass es nun keinen Verteidiger mehr gibt
				building->NoDefender();
			}

		} break;
	default:
		break;
	}
}

/// Wenn ein Heimat-Milit�rgeb�ude bei Missionseins�tzen zerst�rt wurde
void nofDefender::HomeDestroyed()
{
	building = 0;

	switch(state)
	{
	case STATE_DEFENDING_WAITING:
		{
			// Hier muss sofort reagiert werden, da man steht

			attacker = 0;

			// Rumirren
			state = STATE_FIGUREWORK;
			StartWandering();
			Wander();

			

		} break;
	case STATE_DEFENDING_WALKINGTO:
	case STATE_DEFENDING_WALKINGFROM:
		{
			attacker = 0;

			// Rumirren
			StartWandering();
			state = STATE_FIGUREWORK;

		} break;
	case STATE_FIGHTING:
		{
			// Die normale T�tigkeit wird erstmal fortgesetzt (Laufen, K�mpfen, wenn er schon an der Fahne ist
			// wird er auch nicht mehr zur�ckgehen)
		} break;
	default:
		break;
	}
}

void nofDefender::HomeDestroyedAtBegin()
{
	building = 0;

	state = STATE_FIGUREWORK;

	// Rumirren
	StartWandering();
	StartWalking(RANDOM.Rand(__FILE__,__LINE__,obj_id,6));
}


/// Wenn ein Kampf gewonnen wurde
void nofDefender::WonFighting()
{
	// Angreifer tot
	attacker = 0;

	// Ist evtl. unser Heimatgeb�ude zerst�rt?
	if(!building)
	{
		// Rumirren
		state = STATE_FIGUREWORK;
		StartWandering();
		Wander();

		return;
	}

	// Neuen Angreifer rufen
	if( (attacker = building->FindAttackerNearBuilding()) )
	{
		// Ein Angreifer gefunden, dann warten wir auf ihn, bis er kommt
		state = STATE_DEFENDING_WAITING;
	}
	else
	{
		// Kein Angreifer gefunden, dann gehen wir wieder in unser Geb�ude
		state = STATE_DEFENDING_WALKINGFROM;
		StartWalking(1);
		// Angreifer auf 0 setzen, er ist ja tot
		attacker = 0;
	}
}

/// Wenn ein Kampf verloren wurde (Tod)
void nofDefender::LostFighting()
{
	attacker = 0;

	// Geb�ude Bescheid sagen, falls es noch existiert
	if(building)
	{
		building->NoDefender();
		// Ist das ein "normales" Milit�rgeb�ude?
		if(building->GetBuildingType() >= BLD_BARRACKS && building->GetBuildingType() <= BLD_FORTRESS)
		{
			// Wenn ich nicht der lezte Soldat da drinnen war, dann k�nnen noch neue kommen..
			if(static_cast<nobMilitary*>(building)->GetTroopsCount())
				static_cast<nobMilitary*>(building)->RegulateTroops();
		}
	}
}


void nofDefender::AttackerArrested()
{
	// Neuen Angreifer suchen
	if(!(attacker = building->FindAttackerNearBuilding()))
	{
		// Kein Angreifer gefunden, dann gehen wir wieder in unser Geb�ude
		state = STATE_DEFENDING_WALKINGFROM;
		StartWalking(1);
		// Angreifer auf 0 setzen, er ist ja tot
		attacker = 0;
	}
}

/// Sagt den verschiedenen Zielen Bescheid, dass wir doch nicht mehr kommen k�nnen
void nofDefender::InformTargetsAboutCancelling()
{
}


/// The derived classes regain control after a fight of nofActiveSoldier
void nofDefender::FreeFightEnded()
{
	// This is not supposed to happen
	assert(false);
}
