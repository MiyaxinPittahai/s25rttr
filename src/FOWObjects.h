// $Id: FOWObjects.h 6458 2010-05-31 11:38:51Z FloSoft $
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
#ifndef FOWOBJECT_H_INCLUDED
#define FOWOBJECT_H_INCLUDED

#include "GameConsts.h"
#include "MapConsts.h"

class SerializedGameData;

/// Typen f�r die FOW Objekte
enum FOW_Type
{
	FOW_NOTHING,
	FOW_BUILDING,
	FOW_BUILDINGSITE,
	FOW_FLAG,
	FOW_TREE,
	FOW_GRANITE
};

/// Helligkeit der Objekte beim Zeichnen
const unsigned FOW_DRAW_COLOR_BRIGHTNESS = 0x80;
/// Farbe f�r das Zeichnen
const unsigned FOW_DRAW_COLOR = 0xFF808080;

/// Berechnet die dunklere Spielerfarbe zum Zeichnen
unsigned CalcPlayerFOWDrawColor(const unsigned color);

/// Visuelles Objekt im Nebel, nur zur sichtbaren "Erinnerung",
/// was ein bestimmter Spieler gesehen hat
class FOWObject
{
public:

	virtual ~FOWObject();
	/// An x,y zeichnen.
	virtual void Draw(int x, int y) const = 0;
	/// Serialisierungsfunktion.
	virtual void Serialize(SerializedGameData *sgd) const = 0;
	/// Gibt Typ zur�ck
	virtual FOW_Type GetType() const = 0;
};

/// Platzhalter-Objekt, falls dort kein Objekt existiert
class fowNothing : public FOWObject
{
public:

	fowNothing();
	fowNothing(SerializedGameData * sgd);
	void Serialize(SerializedGameData *sgd) const;
	void Draw(int x, int y) const;
	FOW_Type GetType() const { return FOW_NOTHING; }
} const nothing;



/// Geb�ude im Nebel
class fowBuilding : public FOWObject
{
private:
	/// Typ des Geb�udes
	const BuildingType type;
	/// Volk des Geb�udes (muss extra gespeichert werden, da ja auch z.B. fremde Geb�ude erobert werden k�nnen)
	const Nation nation;
public:

	fowBuilding(const BuildingType type, const Nation nation);
	fowBuilding(SerializedGameData * sgd);
	void Serialize(SerializedGameData *sgd) const;
	void Draw(int x, int y) const;
	FOW_Type GetType() const { return FOW_BUILDING; }
};

/// Baustelle
class fowBuildingSite : public FOWObject
{
private:
	/// Wird planiert?
	const bool planing;
	/// Typ des Geb�udes
	const BuildingType type;
	/// Volk des Geb�udes (muss extra gespeichert werden, da ja auch z.B. fremde Geb�ude erobert werden k�nnen)
	const Nation nation;
	/// Gibt den Baufortschritt an, wie hoch das Geb�ude schon gebaut ist, gemessen in 8 Stufen f�r jede verbaute Ware
	const unsigned char build_progress;
public:

	fowBuildingSite(const bool planing, const BuildingType type, const Nation nation, const unsigned char build_progress);
	fowBuildingSite(SerializedGameData * sgd);
	void Serialize(SerializedGameData *sgd) const;
	void Draw(int x, int y) const;
	FOW_Type GetType() const { return FOW_BUILDINGSITE; }
};


/// Flagge
class fowFlag : public FOWObject
{
private:
	/// Besitzer
	const unsigned char player;
	/// Flaggenart
	const FlagType flag_type;

public:

	fowFlag(const unsigned char player, const FlagType flag_type);
	fowFlag(SerializedGameData * sgd);
	void Serialize(SerializedGameData *sgd) const;
	void Draw(int x, int y) const;
	FOW_Type GetType() const { return FOW_FLAG; }
};

/// Baum
class fowTree : public FOWObject
{
private:

	/// Typ des Baumes (also welche Baumart)
	const unsigned char type;
	/// Gr��e des Baumes (0-2, 3 = aufgewachsen!)
	const unsigned char size;

public:

	fowTree(const unsigned char type, const unsigned char size);
	fowTree(SerializedGameData * sgd);
	void Serialize(SerializedGameData *sgd) const;
	void Draw(int x, int y) const;
	FOW_Type GetType() const { return FOW_TREE; }
};

/// Granitblock
class fowGranite : public FOWObject
{
private:

	const GraniteType type; /// Welcher Typ ( gibt 2 )
	const unsigned char state; /// Status, 0 - 5, von sehr wenig bis sehr viel

public:

	fowGranite(const GraniteType type, const unsigned char state);
	fowGranite(SerializedGameData * sgd);
	void Serialize(SerializedGameData *sgd) const;
	void Draw(int x, int y) const;
	FOW_Type GetType() const { return FOW_GRANITE; }
};




#endif // !FOWOBJECT_H_INCLUDED
