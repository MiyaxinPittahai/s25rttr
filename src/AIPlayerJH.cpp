// $Id: AIPlayerJH.cpp 5405 2009-08-13 21:23:32Z jh $
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
#include "AIPlayerJH.h"

#include "GameClientPlayer.h"
#include "GameWorld.h"
#include "GameCommands.h"
#include "GamePlayerList.h"

#include "nobMilitary.h"
#include "nobHQ.h"
#include "noBuildingSite.h"

#include "MapGeometry.h"

#include <iostream>

#include "GameMessages.h"
#include "GameServer.h"

// from Pathfinding.cpp
bool IsPointOK_RoadPath(const GameWorldBase& gwb, const MapCoord x, const MapCoord y, const unsigned char dir, const void *param);

AIPlayerJH::AIPlayerJH(const unsigned char playerid, const GameWorldBase * const gwb, const GameClientPlayer * const player,
		const GameClientPlayerList * const players, const GlobalGameSettings * const ggs,
		const AI::Level level) : AIBase(playerid, gwb, player, players, ggs, level), defeated(false)
{
	currentJob = 0;
	buildingsWanted.resize(BUILDING_TYPES_COUNT);
	militaryBuildingToCheck = 0;
}

/// Wird jeden GF aufgerufen und die KI kann hier entsprechende Handlungen vollziehen
void AIPlayerJH::RunGF(const unsigned gf)
{
	if (gf == 1)
	{
		InitNodes();
		InitResourceMaps();
		InitBuildingsWanted();
		RefreshBuildingCount();
	}

	if (defeated)
		return;

	if (TestDefeat())
		return;

	if ((gf + (playerid * 2)) % 20 == 0)
	{
		RefreshBuildingCount();
		ExecuteAIJob();
	}

	if ((gf + playerid * 10) % 1000 == 0)
	{
		//CheckExistingMilitaryBuildings();
		TryToAttack();
	}

	if ((gf + playerid * 10) % 100 == 0)
	{
		CheckNewMilitaryBuildings();
	}



	if (gf == 5)
	{
		aiJobs.push(new AIJH::BuildJob(this, ChooseMilitaryBuilding(player->hqx, player->hqy)));
		aiJobs.push(new AIJH::BuildJob(this, ChooseMilitaryBuilding(player->hqx, player->hqy)));
		aiJobs.push(new AIJH::BuildJob(this, ChooseMilitaryBuilding(player->hqx, player->hqy)));
		aiJobs.push(new AIJH::BuildJob(this, ChooseMilitaryBuilding(player->hqx, player->hqy)));
		aiJobs.push(new AIJH::BuildJob(this, BLD_FORESTER));
		aiJobs.push(new AIJH::BuildJob(this, BLD_WOODCUTTER));
		aiJobs.push(new AIJH::BuildJob(this, BLD_WOODCUTTER));
		aiJobs.push(new AIJH::BuildJob(this, BLD_QUARRY));
		aiJobs.push(new AIJH::BuildJob(this, BLD_FISHERY));
		
	}

	if (gf == 100)
	{
		Chat(_("Hi, I'm an artifical player and I'm not very good yet!"));
	}
	if (gf == 120)
	{
		Chat(_("And I may crash your game sometimes..."));
	}

	if ((gf % 1000) == 0)
	{
		if (Wanted(BLD_SAWMILL))
		{
			aiJobs.push(new AIJH::BuildJob(this, BLD_SAWMILL));
		}
	}
}

void AIPlayerJH::FindFlags(std::vector<const noFlag*>& flags, unsigned short x, unsigned short y, unsigned short radius)
{
	flags.clear();
	for(MapCoord tx=gwb->GetXA(x,y,0), r=1;r<=radius;tx=gwb->GetXA(tx,y,0),++r)
	{
		MapCoord tx2 = tx, ty2 = y;
		for(unsigned i = 2;i<8;++i)
		{
			for(MapCoord r2=0;r2<r;gwb->GetPointA(tx2,ty2,i%6),++r2)
			{
				if(gwb->GetSpecObj<noFlag>(tx2,ty2))
				{
					flags.push_back(gwb->GetSpecObj<noFlag>(tx2,ty2));
				}
			}
		}
	}
}

// DONT USE THIS METHOD
void AIPlayerJH::ConnectBuildingSites()
{
	// Liste von Baustellen holen
	const std::list<noBuildingSite*> sites = player->GetBuildingSites();

	// Ziel, das m�glichst schnell erreichbar sein soll (TODO m�sste dann evtl. auch Lager/Hafen sein)
	noFlag *targetFlag = gwb->GetSpecObj<nobHQ>(player->hqx, player->hqy)->GetFlag();

	for (std::list<noBuildingSite*>::const_iterator it = sites.begin(); it != sites.end(); ++it)
	{
		unsigned char first_dir = 0xFF;
		// Kann die Baustelle vom HQ erreicht werden? Nein -> Weg bauen!
		if (!gwb->FindPathOnRoads((*it)->GetFlag(), targetFlag, false, NULL, NULL, &first_dir, NULL))
		{
			// Versuchen Weg zu bauen, wenns nicht klappt, Baustelle abrei�en
			//if (!ConnectFlagToRoadSytem((*it)->GetFlag()))
			{
				gcs.push_back(new gc::DestroyBuilding((*it)->GetX(), (*it)->GetY()));
			}
			//else
			{
				// Besser immer nur ein Weg bauen (TODO Wenn berechnete Wege sich kreuzen, geht was kaputt)
				return;
			}
		}
	}
}

bool AIPlayerJH::ConnectFlagToRoadSytem(const noFlag *flag, std::vector<unsigned char>& route)
{
	// Radius in dem nach w�rdigen Fahnen gesucht wird
	const unsigned short maxRoadLength = 10;

	// Ziel, das m�glichst schnell erreichbar sein soll (TODO m�sste dann evtl. auch Lager/Hafen sein)
	noFlag *targetFlag = gwb->GetSpecObj<nobHQ>(player->hqx, player->hqy)->GetFlag();

	// Flaggen in der Umgebung holen
	std::vector<const noFlag*> flags;
	FindFlags(flags, flag->GetX(), flag->GetY(), maxRoadLength);

	unsigned shortest = 0;
	unsigned int shortestLength = 99999;
	bool found = false;
	
	// Jede Flagge testen...
	for(unsigned i=0; i<flags.size(); ++i)
	{
		//std::vector<unsigned char> new_route;
		route.clear();
		unsigned int length;
		Param_RoadPath prp = { false };
		
		// Gibts �berhaupt einen Pfad zu dieser Flagge
		bool path_found = gwb->FindFreePath(flag->GetX(),flag->GetY(),
		                  flags[i]->GetX(),flags[i]->GetY(),false,100,&route,&length,NULL,NULL,IsPointOK_RoadPath,NULL, &prp);

		// Wenn ja, dann gucken ob dieser Pfad m�glichst kurz zum "h�heren" Ziel (HQ im Moment) ist
		if (path_found)
		{
			unsigned char first_dir;
			unsigned int hqlength = 0;

			// F�hrt manchmal zu Problemen, tempor�r raus; TODO was anderes �berlegen (ganz ohne ist auch doof)

			// Strecke von der potenziellen Zielfahne bis zum HQ
			bool hq_path_found = gwb->FindPathOnRoads(flags[i], targetFlag, false, NULL, &hqlength, &first_dir, NULL);

			// Gew�hlte Fahne hat leider auch kein Anschlu� ans HQ, zu schade!
			if (!hq_path_found)
				// Und ist auch nicht zuf�llig die HQ-Flagge selber...
				if (flags[i]->GetX() != targetFlag->GetX() || flags[i]->GetY() != targetFlag->GetY())
					continue;
			

			// Sind wir mit der Fahne schon verbunden? Einmal reicht!
			bool alreadyConnected = gwb->FindPathOnRoads(flags[i], flag, false, NULL, NULL, &first_dir, NULL);
			if (alreadyConnected)
				continue;
			
			// Ansonsten haben wir einen Pfad!
			found = true;
			
			// K�rzer als der letzte? Nehmen!
			if (length + hqlength < shortestLength)
			{
				shortest = i;
				shortestLength = length+hqlength;
			}
		}
	}

	if (found)
	{
		return BuildRoad(flag, flags[shortest]);
	}
	return false;
}

bool AIPlayerJH::BuildRoad(const noRoadNode *start, const noRoadNode *target)
{
	// Gucken obs einen Weg gibt
	std::vector<unsigned char> route;
	Param_RoadPath prp = { false };

	bool foundPath = gwb->FindFreePath(start->GetX(),start->GetY(),
	                  target->GetX(),target->GetY(),false,100,&route,NULL,NULL,NULL,IsPointOK_RoadPath,NULL, &prp);

	// Wenn Pfad gefunden, Befehl zum Stra�e bauen und Flagen setzen geben
	if (foundPath)
	{
		MapCoord x = start->GetX();
		MapCoord y = start->GetY();
		gcs.push_back(new gc::BuildRoad(x, y, false, route));


		// Flaggen auf der Stra�e setzen
		for(unsigned i=0; i<route.size(); ++i)
		{
			gwb->GetPointA(x, y, route[i]);
			// Alle zwei Teilst�cke versuchen eine Flagge zu bauen
			if (i % 2 == 1) //&& gwb->GetNode(x,y).bq >= BQ_FLAG) // TODO warum geht nicht mit bq-Abfrage?
			{
				gcs.push_back(new gc::SetFlag(x,y));
			}
		}
		return true;
	}
	return false;
}

bool AIPlayerJH::TestDefeat()
{
	const nobHQ *hq = gwb->GetSpecObj<nobHQ>(player->hqx, player->hqy);
	if (!hq)
	{
		defeated = true;
		gcs.push_back(new gc::Surrender());
		Chat(_("Oh, no, you destroyed my headquarter! I surrender!"));
		return true;
	}
	return false;
}

AIJH::Resource AIPlayerJH::CalcResource(MapCoord x, MapCoord y)
{
	AIJH::Resource res = AIJH::NOTHING;

	// Unterirdische Ressourcen
	unsigned char subres = gwb->GetNode(x,y).resources;
	if (subres > 0x40+0*8 && subres < 0x48+0*8)
		res = AIJH::COAL;
	else if (subres > 0x40+1*8 && subres < 0x48+1*8)
		res = AIJH::IRONORE;
	else if (subres > 0x40+2*8 && subres < 0x48+2*8)
		res = AIJH::GOLD;
	else if (subres > 0x40+3*8 && subres < 0x48+3*8)
		res = AIJH::GRANITE;

	if (subres > 0x80 && subres < 0x90)
		res = AIJH::FISH;

	// �berirdische Resourcen
	NodalObjectType no = gwb->GetNO(x,y)->GetType();

	if (no == NOP_TREE)
		res = AIJH::WOOD;
	else if(no == NOP_GRANITE)
		res = AIJH::STONES;
	else if(res == AIJH::NOTHING && (no == NOP_NOTHING || no == NOP_ENVIRONMENT)) // TODO getterain
		res = AIJH::PLANTSPACE;

	return res;
}

void AIPlayerJH::InitNodes()
{
	unsigned short width = gwb->GetWidth();
	unsigned short height = gwb->GetHeight();
	unsigned int playerID = player->getPlayerID();

	nodes.resize(width * height);

	for (unsigned short y=0; y<height; ++y)
	{
		for (unsigned short x=0; x<width; ++x)
		{
			unsigned i = x + y * width;

			// BuildingQuality + Owner-Kram + Borderland"resource"
			assert(player->GetMilitaryBuildings().size() == 0); // TODO f�r savegame laden muss das weg

			// Nur f�r HQ:
			unsigned distance = CalcDistance(player->hqx, player->hqy, x, y);
			if (distance < MILITARY_RADIUS[4])
			{
				nodes[i].owned = true;
				nodes[i].bq = gwb->CalcBQ(x, y, playerID);

				Param_RoadPath prp;
				prp.boat_road = false;

				if (gwb->FindFreePath(x,y,
					gwb->GetXA(player->hqx, player->hqy, 4),
					gwb->GetYA(player->hqx, player->hqy, 4),					
					false,50,NULL,NULL,NULL,NULL,IsPointOK_RoadPath,NULL, &prp))
				{
					nodes[i].reachable = true;
				}
				else
				{
					nodes[i].reachable = false;
				}
			}
			else
			{
				nodes[i].owned = false;
				nodes[i].bq = BQ_NOTHING;
				nodes[i].reachable = false;
			}
			nodes[i].res = CalcResource(x, y);

			nodes[i].border = (gwb->GetNode(x, y).boundary_stones[0] != 0);
			//if (nodes[i].border)
				//std::cout << x << " / " << y << " Border true" << std::endl;
		}
	}
}

void AIPlayerJH::InitResourceMaps()
{
	unsigned short width = gwb->GetWidth();
	unsigned short height = gwb->GetHeight();

	resourceMaps.resize(AIJH::RES_TYPE_COUNT);
	for (unsigned res=0; res<AIJH::RES_TYPE_COUNT; ++res)
	{
		resourceMaps[res].resize(width * height);
		for (unsigned short y=0; y<height; ++y)
		{
			for (unsigned short x=0; x<width; ++x)
			{
				unsigned i = y * width + x;
				//resourceMaps[res][i] = 0;
				if (nodes[i].res == (AIJH::Resource)res && (AIJH::Resource)res != AIJH::BORDERLAND)
				{
					ChangeResourceMap(x, y, AIJH::RES_RADIUS[res], resourceMaps[res], 1);
				}

				// Grenzgebiet"ressource"
				else if (nodes[i].border && (AIJH::Resource)res == AIJH::BORDERLAND)
				{
					ChangeResourceMap(x, y, AIJH::RES_RADIUS[AIJH::BORDERLAND], resourceMaps[AIJH::BORDERLAND], 1);
				}
			}
		}
	}
}

void AIPlayerJH::ChangeResourceMap(MapCoord x, MapCoord y, unsigned radius, std::vector<int> &resMap, int value)
{
	unsigned short width = gwb->GetWidth();
	//unsigned short height = gwb->GetHeight();

	/*
	int startX = x - radius;
	if (startX < 0)
		startX = 0;

	int startY = y - radius;
	if (startY < 0)
		startY = 0;

	int endX = x + radius;
	if (endX >= width)
		endX = width - 1;

	int endY = y + radius;
	if (endY >= height)
		endY = height - 1;

	for(int ty = startY; ty < endY; ++ty)
	{
		for(int tx = startX; tx < endX; ++tx)
		{
			int val = radius - abs(x - tx) - abs(y - ty);
			if (val >= 0)
			  resMap[tx + ty * width] += value;
		}
	}
	*/

	resMap[x + y * width] = value * radius;

	for(MapCoord tx=gwb->GetXA(x,y,0), r=1;r<=radius;tx=gwb->GetXA(tx,y,0),++r)
	{
		MapCoord tx2 = tx, ty2 = y;
		for(unsigned i = 2;i<8;++i)
		{
			for(MapCoord r2=0;r2<r;gwb->GetPointA(tx2,ty2,i%6),++r2)
			{
				unsigned i = tx2 + ty2 * width;
				resMap[i] += value * (radius-r);
			}
		}
	}


}

bool AIPlayerJH::FindGoodPosition(MapCoord &x, MapCoord &y, AIJH::Resource res, int threshold, BuildingQuality size, int radius, bool inTerritory)
{
	unsigned short width = gwb->GetWidth();
	unsigned short height = gwb->GetHeight();

	if (x < 0 || x >= width || y < 0 || y >= height)
	{
		x = player->hqx;
		y = player->hqy;
	}

	// TODO was besseres w�r sch�n ;)
	if (radius == -1)
		radius = 30;

	for(MapCoord tx=gwb->GetXA(x,y,0), r=1;r<=radius;tx=gwb->GetXA(tx,y,0),++r)
	{
		MapCoord tx2 = tx, ty2 = y;
		for(unsigned i = 2;i<8;++i)
		{
			for(MapCoord r2=0;r2<r;gwb->GetPointA(tx2,ty2,i%6),++r2)
			{
				unsigned i = tx2 + ty2 * width;
				if (resourceMaps[res][i] >= threshold)
				{
					if (inTerritory && !nodes[i].owned)
						continue;
					if ( (nodes[i].bq >= size && nodes[i].bq < BQ_MINE) // normales Geb�ude
						|| (nodes[i].bq == size))	// auch Bergwerke
					{
						x = tx2;
						y = ty2;
						return true;
					}
				}
			}
		}
	}
	return false;
}


bool AIPlayerJH::FindBestPosition(MapCoord &x, MapCoord &y, AIJH::Resource res, BuildingQuality size, int minimum, int radius, bool inTerritory)
{
	unsigned short width = gwb->GetWidth();
	unsigned short height = gwb->GetHeight();

	if (x < 0 || x >= width || y < 0 || y >= height)
	{
		x = player->hqx;
		y = player->hqy;
	}

	// TODO was besseres w�r sch�n ;)
	if (radius == -1)
		radius = 30;

	int best_x, best_y, best_value;
	best_value = -1;

	for(MapCoord tx=gwb->GetXA(x,y,0), r=1;r<=radius;tx=gwb->GetXA(tx,y,0),++r)
	{
		MapCoord tx2 = tx, ty2 = y;
		for(unsigned i = 2;i<8;++i)
		{
			for(MapCoord r2=0;r2<r;gwb->GetPointA(tx2,ty2,i%6),++r2)
			{
				unsigned i = tx2 + ty2 * width;
				if (resourceMaps[res][i] > best_value)
				{
					if (!nodes[i].reachable || (inTerritory && !nodes[i].owned))
						continue;
					if ( (nodes[i].bq >= size && nodes[i].bq < BQ_MINE) // normales Geb�ude
						|| (nodes[i].bq == size))	// auch Bergwerke
					{
						best_x = tx2;
						best_y = ty2;
						best_value = resourceMaps[res][i];
					}
				}
			}
		}
	}

	if (best_value >= minimum)
	{
		x = best_x;
		y = best_y;
		return true;
	}
	return false;
}

void AIPlayerJH::UpdateNodesAround(MapCoord x, MapCoord y, unsigned radius)
{
	unsigned width = gwb->GetWidth();
	//const nobMilitary *mil = gwb->GetSpecObj<nobMilitary>(x,y);
	//assert(mil);

	for(MapCoord tx=gwb->GetXA(x,y,0), r=1;r<=radius;tx=gwb->GetXA(tx,y,0),++r)
	{
		MapCoord tx2 = tx, ty2 = y;
		for(unsigned i = 2;i<8;++i)
		{
			for(MapCoord r2=0;r2<r;gwb->GetPointA(tx2,ty2,i%6),++r2)
			{
				unsigned i = tx2 + ty2 * width;

				//unsigned distance = CalcDistance(tx2, ty2, x, y);
				nodes[i].owned = (gwb->GetNode(tx2, ty2).owner == playerid + 1);

				if (nodes[i].owned)
				{
					nodes[i].bq = gwb->CalcBQ(tx2, ty2, playerid);

					Param_RoadPath prp;
					prp.boat_road = false;

					if (gwb->FindFreePath(tx2,ty2,
						gwb->GetXA(x, y, 4),
						gwb->GetYA(x, y, 4),					
						false,50,NULL,NULL,NULL,NULL,IsPointOK_RoadPath,NULL, &prp))
					{
						nodes[i].reachable = true;
					}
					else
					{
						nodes[i].reachable = false;
					}
				}
				else
				{
					nodes[i].owned = false;
					nodes[i].bq = BQ_NOTHING;
					nodes[i].reachable = false;
				}

				AIJH::Resource res = CalcResource(tx2, ty2);
				if (res != nodes[i].res)
				{
					// Altes entfernen:
					if (nodes[i].res != AIJH::NOTHING)
						ChangeResourceMap(tx2, ty2, AIJH::RES_RADIUS[nodes[i].res], resourceMaps[nodes[i].res], -1);
					// Neues Hinzuf�gen:
					if (res != AIJH::NOTHING)
						ChangeResourceMap(tx2, ty2, AIJH::RES_RADIUS[res], resourceMaps[res], 1);

					nodes[i].res = res;
				}

				bool borderland = (gwb->GetNode(tx2, ty2).boundary_stones[0] != 0);
				if (borderland != nodes[i].border)
				{
					if (borderland)
					{
						//std::cout << tx2 << " / " << ty2 << " Border dazugekommen" << std::endl;
						ChangeResourceMap(tx2, ty2, AIJH::RES_RADIUS[AIJH::BORDERLAND], resourceMaps[AIJH::BORDERLAND], 1);
					}
					else
					{
						//std::cout << tx2 << " / " << ty2 << " Border verschwunden" << std::endl;
						ChangeResourceMap(tx2, ty2, AIJH::RES_RADIUS[AIJH::BORDERLAND], resourceMaps[AIJH::BORDERLAND], -1);
					}
				}

			}
		}
	}
}

void AIPlayerJH::ExecuteAIJob()
{
	if (currentJob)
	{
		if (currentJob->GetStatus() == AIJH::JOB_FINISHED)
		{
			delete currentJob;
			currentJob = 0;
		}
	}
	if (currentJob)
	{
		if (currentJob->GetStatus() == AIJH::JOB_FAILED)
		{
			// TODO fehlerbehandlung?
			//std::cout << "Job failed." << std::endl;
			delete currentJob;
			currentJob = 0;
		}
	}

	if (!currentJob)
	{
		if (aiJobs.size() > 0)
		{
			currentJob = aiJobs.front();
			aiJobs.pop();
		}
	}

	if (currentJob)
		currentJob->ExecuteJob();
}

bool AIPlayerJH::IsConnectedToRoadSystem(const noFlag *flag)
{
	// TODO target ist atm immer das HQ
	const noFlag *targetFlag = gwb->GetSpecObj<noFlag>(gwb->GetXA(player->hqx, player->hqy, 4), gwb->GetYA(player->hqx, player->hqy, 4));
	if (targetFlag)
		return gwb->FindPathOnRoads(flag, targetFlag, false, NULL, NULL, NULL, NULL);
	else 
		return false;
}

void AIPlayerJH::RecalcBQAround(const MapCoord x, const MapCoord y)
{
	// Drumherum BQ neu berechnen, da diese sich ja jetzt h�tten �ndern k�nnen
	unsigned index = x + y * gwb->GetWidth();

	nodes[index].bq = gwb->CalcBQ(x,y,playerid);
	for(unsigned char i = 0;i<6;++i)
	{
		index = gwb->GetXA(x,y,i) + gwb->GetYA(x,y,i) * gwb->GetWidth();
		nodes[index].bq = gwb->CalcBQ(gwb->GetXA(x,y,i), gwb->GetYA(x,y,i),playerid);
	}
	for(unsigned i = 0;i<12;++i)
	{
		index = gwb->GetXA2(x,y,i) + gwb->GetYA2(x,y,i) * gwb->GetWidth();
		nodes[index].bq = gwb->CalcBQ(gwb->GetXA2(x,y,i),gwb->GetYA2(x,y,i),playerid);
	}

}

void AIPlayerJH::CheckNewMilitaryBuildings()
{
	for (std::list<Coords>::iterator it = milBuildingSites.begin(); it != milBuildingSites.end(); it++)
	{
		const nobMilitary *mil;
		if ((mil = gwb->GetSpecObj<nobMilitary>((*it).x, (*it).y)))
		{
			if (!mil->IsNewBuilt())
			{
				HandleNewMilitaryBuilingOccupied(*it);
				milBuildings.push_back(Coords(mil->GetX(), mil->GetY()));
				milBuildingSites.erase(it);
				break;
			}
		}
	}
}

BuildingType AIPlayerJH::ChooseMilitaryBuilding(MapCoord x, MapCoord y)
{
	BuildingType bld = BLD_BARRACKS;

	if ((rand() % 3) == 0)
		bld = BLD_GUARDHOUSE;

	list<nobBaseMilitary*> military;
	gwb->LookForMilitaryBuildings(military, x, y, 2);
	for(list<nobBaseMilitary*>::iterator it = military.begin();it.valid();++it)
	{
		unsigned distance = CalcDistance((*it)->GetX(), (*it)->GetY(), x, y);

		// Pr�fen ob Feind in der N�he
		if ((*it)->GetPlayer() != player->getPlayerID() && distance < 30)
		{
			if ((rand() % 10) == 0)
				bld = BLD_CATAPULT;
			if ((rand() % 2) == 0)
				bld = BLD_FORTRESS;
			else
				bld = BLD_WATCHTOWER;

			break;
		}
	}

	return bld;
}


bool AIPlayerJH::SimpleFindPosition(MapCoord &x, MapCoord &y, BuildingQuality size, int radius)
{
	unsigned short width = gwb->GetWidth();
	unsigned short height = gwb->GetHeight();

	if (x < 0 || x >= width || y < 0 || y >= height)
	{
		x = player->hqx;
		y = player->hqy;
	}

	// TODO was besseres w�r sch�n ;)
	if (radius == -1)
		radius = 30;

	for(MapCoord tx=gwb->GetXA(x,y,0), r=1;r<=radius;tx=gwb->GetXA(tx,y,0),++r)
	{
		MapCoord tx2 = tx, ty2 = y;
		for(unsigned i = 2;i<8;++i)
		{
			for(MapCoord r2=0;r2<r;gwb->GetPointA(tx2,ty2,i%6),++r2)
			{
				unsigned i = tx2 + ty2 * width;

				if (!nodes[i].reachable || !nodes[i].owned)
					continue;
				if ( (nodes[i].bq >= size && nodes[i].bq < BQ_MINE) // normales Geb�ude
					|| (nodes[i].bq == size))	// auch Bergwerke
				{
					x = tx2;
					y = ty2;
					return true;
				}
			}
		}
	}

	return false;
}

void AIPlayerJH::HandleNewMilitaryBuilingOccupied(const Coords& coords)
{
	MapCoord x = coords.x;
	MapCoord y = coords.y;
	UpdateNodesAround(x, y, 11); // todo: fix radius
	RefreshBuildingCount();

	aiJobs.push(new AIJH::BuildJob(this, ChooseMilitaryBuilding(x, y), x, y));
	aiJobs.push(new AIJH::BuildJob(this, ChooseMilitaryBuilding(x, y), x, y));
	aiJobs.push(new AIJH::BuildJob(this, ChooseMilitaryBuilding(x, y), x, y));

	// Tempor�r only
	aiJobs.push(new AIJH::BuildJob(this, BLD_FORESTER, x, y));
	aiJobs.push(new AIJH::BuildJob(this, BLD_WOODCUTTER, x, y));

	aiJobs.push(new AIJH::BuildJob(this, BLD_QUARRY, x, y));

	aiJobs.push(new AIJH::BuildJob(this, BLD_GOLDMINE, x, y));
	aiJobs.push(new AIJH::BuildJob(this, BLD_COALMINE, x, y));
	aiJobs.push(new AIJH::BuildJob(this, BLD_IRONMINE, x, y));

	aiJobs.push(new AIJH::BuildJob(this, BLD_SAWMILL, x, y));

	aiJobs.push(new AIJH::BuildJob(this, BLD_IRONSMELTER, x, y));
	aiJobs.push(new AIJH::BuildJob(this, BLD_MINT, x, y));
	aiJobs.push(new AIJH::BuildJob(this, BLD_ARMORY, x, y));

	aiJobs.push(new AIJH::BuildJob(this, BLD_FISHERY, x, y));

	aiJobs.push(new AIJH::BuildJob(this, BLD_HUNTER, x, y));
}

void AIPlayerJH::HandleRetryMilitaryBuilding(const Coords& coords)
{
	MapCoord x = coords.x;
	MapCoord y = coords.y;
	UpdateNodesAround(x, y, 11); // todo: fix radius
	RefreshBuildingCount();

	aiJobs.push(new AIJH::BuildJob(this, ChooseMilitaryBuilding(x, y), x, y));
	aiJobs.push(new AIJH::BuildJob(this, ChooseMilitaryBuilding(x, y), x, y));
	aiJobs.push(new AIJH::BuildJob(this, ChooseMilitaryBuilding(x, y), x, y));

	// Tempor�r only
	aiJobs.push(new AIJH::BuildJob(this, BLD_WOODCUTTER, x, y));

	aiJobs.push(new AIJH::BuildJob(this, BLD_QUARRY, x, y));

	aiJobs.push(new AIJH::BuildJob(this, BLD_GOLDMINE, x, y));
	aiJobs.push(new AIJH::BuildJob(this, BLD_COALMINE, x, y));
	aiJobs.push(new AIJH::BuildJob(this, BLD_IRONMINE, x, y));

	aiJobs.push(new AIJH::BuildJob(this, BLD_SAWMILL, x, y));

	aiJobs.push(new AIJH::BuildJob(this, BLD_IRONSMELTER, x, y));
	aiJobs.push(new AIJH::BuildJob(this, BLD_MINT, x, y));
	aiJobs.push(new AIJH::BuildJob(this, BLD_ARMORY, x, y));
}


void AIPlayerJH::RefreshBuildingCount()
{
	player->GetBuildingCount(buildingCounts);

	buildingsWanted[BLD_SAWMILL] = 1 + GetBuildingCount(BLD_WOODCUTTER) / 2;
	buildingsWanted[BLD_IRONSMELTER] = GetBuildingCount(BLD_IRONMINE);
	buildingsWanted[BLD_MINT] = GetBuildingCount(BLD_GOLDMINE);
	buildingsWanted[BLD_ARMORY] = GetBuildingCount(BLD_IRONSMELTER);

	buildingsWanted[BLD_MILL] = GetBuildingCount(BLD_FARM) / 2;
	buildingsWanted[BLD_BAKERY] = GetBuildingCount(BLD_MILL);

	buildingsWanted[BLD_WELL] = GetBuildingCount(BLD_BAKERY) + GetBuildingCount(BLD_PIGFARM) 
		+ GetBuildingCount(BLD_DONKEYBREEDER) + GetBuildingCount(BLD_BREWERY);
}

void AIPlayerJH::InitBuildingsWanted()
{
	buildingsWanted[BLD_FORESTER] = 1;
	buildingsWanted[BLD_SAWMILL] = 1;
	buildingsWanted[BLD_WOODCUTTER] = 12;
	buildingsWanted[BLD_QUARRY] = 6;
	buildingsWanted[BLD_GRANITEMINE] = 0;
	buildingsWanted[BLD_COALMINE] = 3;
	buildingsWanted[BLD_IRONMINE] = 1;
	buildingsWanted[BLD_GOLDMINE] = 1;
	buildingsWanted[BLD_CATAPULT] = 5;
	buildingsWanted[BLD_FISHERY] = 6;
	buildingsWanted[BLD_QUARRY] = 6;
	buildingsWanted[BLD_HUNTER] = 2;
	buildingsWanted[BLD_FARM] = 8;
}

unsigned AIPlayerJH::GetBuildingCount(BuildingType type)
{
	return buildingCounts.building_counts[type] + buildingCounts.building_site_counts[type];
}

bool AIPlayerJH::Wanted(BuildingType type)
{
	if (type >= BLD_BARRACKS && type <= BLD_FORTRESS)
		return true;
	return GetBuildingCount(type) < buildingsWanted[type];
}

void AIPlayerJH::CheckExistingMilitaryBuildings()
{
	if (milBuildings.size() == 0)
		return;
	std::list<Coords>::iterator it = milBuildings.begin();

	if (militaryBuildingToCheck >= milBuildings.size())
		militaryBuildingToCheck = 0;

	std::advance(it,militaryBuildingToCheck);

	assert(it != milBuildings.end());

	HandleRetryMilitaryBuilding(*it);

	const nobBaseMilitary *mil;

	if (!(mil = gwb->GetSpecObj<nobBaseMilitary>((*it).x, (*it).y)))
	{
		// Geb�ude wurde wohl zerst�rt
		milBuildings.erase(it);
		return;
	}

	if (mil->GetPlayer() != playerid)
	{
		// Geb�ude geh�rt nicht mehr uns O_o
		milBuildings.erase(it);
		return;
	}

	militaryBuildingToCheck++;	
}


void AIPlayerJH::Chat(std::string message)
{
	GameMessage_Server_Chat chat = GameMessage_Server_Chat(playerid,CD_ALL,message);
	GameServer::inst().AIChat(chat);
}

void AIPlayerJH::TryToAttack() 
{
	for (std::list<Coords>::iterator it = milBuildings.begin(); it != milBuildings.end(); it++)
	{
		const nobMilitary *mil;
		if ((mil = gwb->GetSpecObj<nobMilitary>((*it).x, (*it).y)))
		{
			if (mil->GetFrontierDistance() != 2)
				continue;

			list<nobBaseMilitary *> buildings;
			gwb->LookForMilitaryBuildings(buildings,(*it).x, (*it).y,2);
			for(list<nobBaseMilitary*>::iterator it2 = buildings.begin();it2.valid();++it2)
			{
				if (CalcDistance((*it).x, (*it).y, (*it2)->GetX(), (*it2)->GetY()) < BASE_ATTACKING_DISTANCE 
					&& player->IsPlayerAttackable((*it2)->GetPlayer()) && gwb->GetNode((*it2)->GetX(), (*it2)->GetY()).fow[playerid].visibility == VIS_VISIBLE)
				{
					gcs.push_back(new gc::Attack((*it2)->GetX(), (*it2)->GetY(), mil->GetTroopsCount() - 1, true));
				}
			}
		}
	}
}
