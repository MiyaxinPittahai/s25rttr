// $Id: Loader.h 5125 2009-06-26 20:10:42Z OLiver $
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
#ifndef LOADER_H_INCLUDED
#define LOADER_H_INCLUDED

#pragma once

#include "Singleton.h"
#include "../libsiedler2/src/libsiedler2.h"

/// Loader Klasse.
class Loader : public Singleton<Loader>
{
public:
	/// Konstruktor von @p Loader.
	Loader(void);
	/// Desktruktor von @p Loader.
	~Loader(void);

	static char *GetFilePath(char *destination, const char *constant);

	/// L�dt alle allgemeinen Dateien.
	bool LoadFiles(void);
	/// L�dt die Men�dateien.
	bool LoadMenu(void);
	/// L�dt die Spieldateien.
	bool LoadGame(unsigned char gfxset, bool *nations);
	/// L�dt das Terrain.
	bool LoadTerrain(unsigned char gfxset);
	/// L�dt alle Textfiles.
	bool LoadTXTs();

	/// L�dt die Settings.
	bool LoadSettings();
	/// Speichert die Settings.
	bool SaveSettings();

protected:
	/// L�dt die Paletten.
	inline bool LoadPalettes();
	/// L�dt die Hintergrunddateien.
	inline bool LoadBackgrounds();
	/// L�dt alle Sounds.
	inline bool LoadSounds();

private:
	bool LoadFile(const char *pfad, const libsiedler2::ArchivItem_Palette *palette, libsiedler2::ArchivInfo *archiv);
	void ExtractTexture(libsiedler2::ArchivInfo *source, libsiedler2::ArchivInfo *destination, Rect &rect);
	void ExtractAnimatedTexture(libsiedler2::ArchivInfo *source, libsiedler2::ArchivInfo *destination, Rect &rect, unsigned char color_count, unsigned char start_index);

public:
	static const unsigned int NATION_COUNT = 4;

	libsiedler2::ArchivInfo io_dat;
	libsiedler2::ArchivInfo resource_dat;
	libsiedler2::ArchivInfo sound_lst;
	libsiedler2::ArchivInfo sng_lst;
	libsiedler2::ArchivInfo outline_fonts_lst;
	libsiedler2::ArchivInfo rttr_lst;

	libsiedler2::ArchivInfo palettes;
	libsiedler2::ArchivInfo backgrounds;
	libsiedler2::ArchivInfo pics;


	libsiedler2::ArchivInfo client_txt;
	libsiedler2::ArchivInfo lang_txt;

	libsiedler2::ArchivInfo textures;
	libsiedler2::ArchivInfo water;
	libsiedler2::ArchivInfo lava;
	libsiedler2::ArchivInfo borders;
	libsiedler2::ArchivInfo roads;
	libsiedler2::ArchivInfo roads_points;

	libsiedler2::ArchivInfo map_lst;
	libsiedler2::ArchivInfo rombobs_lst;
	libsiedler2::ArchivInfo nation_bobs[NATION_COUNT];
	libsiedler2::ArchivInfo nation_icons[NATION_COUNT];
	libsiedler2::ArchivInfo misxbobs[6];
	libsiedler2::ArchivInfo boat_lst;
	libsiedler2::ArchivInfo boot_lst;

	libsiedler2::ArchivInfo carrier_bob;
	libsiedler2::ArchivInfo jobs_bob;

	libsiedler2::ArchivInfo settings;
};

///////////////////////////////////////////////////////////////////////////////
// Makros / Defines
#define LOADER Loader::inst()

#endif // LOADER_H_INCLUDED
