// $Id: noCharburnerPile.cpp 6721 2010-09-10 09:33:56Z OLiver $
//
// Copyright (c) 2005 - 2010 Settlers Freaks (sf-team at siedler25.org)
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
#include "noCharburnerPile.h"

#include "Loader.h"
#include "GameClient.h"
#include "Random.h"
#include "GameWorld.h"
#include "SerializedGameData.h"
#include "noEnvObject.h"

///////////////////////////////////////////////////////////////////////////////
// Makros / Defines
#if defined _WIN32 && defined _DEBUG && defined _MSC_VER
	#define new new(_NORMAL_BLOCK, THIS_FILE, __LINE__)
	#undef THIS_FILE
	static char THIS_FILE[] = __FILE__;
#endif


/// Length of the smoldering
const unsigned SMOLDERING_LENGTH = 3000;

/// Work steps for the construction of the wood pile and the cover
const unsigned short CONSTRUCTION_WORKING_STEPS[2] = {6,6};
/// Work steps for one graphical step during the remove of the cover
const unsigned short REMOVECOVER_WORK_STEPS = 1;
/// Work steps for one graphical step during the "harvest"
const unsigned short HARVEST_WORK_STEPS = 1;

noCharburnerPile::noCharburnerPile(const unsigned short x, const unsigned short y) : noCoordBase(NOP_CHARBURNERPILE,x,y),
state(STATE_WOOD), step(0), sub_step(1), event(NULL)
{
}

noCharburnerPile::~noCharburnerPile()
{
	
}

void noCharburnerPile::Destroy_noGrainfield()
{
	em->RemoveEvent(event);

	// Bauplštze drumrum neu berechnen
	gwg->RecalcBQAroundPoint(x,y);

	Destroy_noCoordBase();
}

void noCharburnerPile::Serialize_noCharburnerPile(SerializedGameData * sgd) const
{
	Serialize_noCoordBase(sgd);

	sgd->PushUnsignedChar(static_cast<unsigned char>(state));
	sgd->PushObject(event,true);
}

noCharburnerPile::noCharburnerPile(SerializedGameData * sgd, const unsigned obj_id) : noCoordBase(sgd,obj_id),
state(State(sgd->PopUnsignedChar())),
event(sgd->PopObject<EventManager::Event>(GOT_EVENT))
{
}

void noCharburnerPile::Draw( int x,	int y)
{
	switch(state)
	{
	case STATE_WOOD:
		{
			// Draw sand on which the wood stack is constructed
			LOADER.GetImageN("charburner_bobs",25)->Draw(x,y);

			glArchivItem_Bitmap * image;
			if(step == 0)
				image = LOADER.GetImageN("charburner_bobs",26);
			else 
			{
				image = LOADER.GetImageN("charburner_bobs",28);

				// Draw wood pile beneath the cover
				 LOADER.GetImageN("charburner_bobs",26)->Draw(x,y);
			}

			unsigned short progress = sub_step*image->getHeight() / CONSTRUCTION_WORKING_STEPS[step];
			unsigned short height = image->getHeight() - progress;
			if(progress != 0)
				image->Draw(x, y+height, 0, 0, 0, height, 0, progress);
		} return;
	case STATE_SMOLDERING:
		{
			LOADER.GetImageN("charburner_bobs",27+GameClient::inst().
				GetGlobalAnimation(2,10,1,obj_id+this->x*10+this->y*10))->Draw(x,y);
		} return;
	case STATE_REMOVECOVER:
		{
			LOADER.GetImageN("charburner_bobs",28+step)->Draw(x,y);
		} return;
	case STATE_HARVEST:
		{
			LOADER.GetImageN("charburner_bobs",34+step)->Draw(x,y);

		} return;
	default: return;
	}
}

void noCharburnerPile::HandleEvent(const unsigned int id)
{
	// Smoldering is over 
	// Pile is ready for the remove of the cover
	state = STATE_REMOVECOVER;
	event = NULL;
}


/// Charburner has worked on it --> Goto next step
void noCharburnerPile::NextStep()
{
	switch(state)
	{
	default: return;
	case STATE_WOOD:
		{
			++sub_step;
			if(sub_step == CONSTRUCTION_WORKING_STEPS[step])
			{
				++step;
				sub_step = 0;

				// Reached new state?
				if(step == 2)
				{
					step = 0;
					state = STATE_SMOLDERING;
					event = em->AddEvent(this,SMOLDERING_LENGTH,0);
				}
			}

		} return;
	case STATE_REMOVECOVER:
		{
			++sub_step;
			if(sub_step == REMOVECOVER_WORK_STEPS)
			{
				++step;
				sub_step = 0;

				// Reached new state?
				if(step == 6)
				{
					state = STATE_HARVEST;
					step = 0;
				}
			}

		} return;
	case STATE_HARVEST:
		{
			++sub_step;
			if(sub_step == HARVEST_WORK_STEPS)
			{
				++step;
				sub_step = 0;

				// Reached new state?
				if(step == 6)
				{
					// Add an empty pile as environmental object
					gwg->SetNO(new noEnvObject(x,y,40,6),x,y);
					em->AddToKillList(this);

					// BQ drumrum neu berechnen
					gwg->RecalcBQAroundPoint(x,y);
				}
			}
		} return;

	}
}


noCharburnerPile::WareType noCharburnerPile::GetNeededWareType() const
{
	if(sub_step % 2 == 0)
		return WT_WOOD;
	else 
		return WT_GRAIN;
}
