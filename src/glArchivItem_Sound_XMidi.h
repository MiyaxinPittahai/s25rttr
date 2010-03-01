// $Id: glArchivItem_Sound_XMidi.h 5853 2010-01-04 16:14:16Z FloSoft $
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
#ifndef GLARCHIVITEM_SOUND_XMIDI_H_INCLUDED
#define GLARCHIVITEM_SOUND_XMIDI_H_INCLUDED

#pragma once

class glArchivItem_Sound_XMidi : public libsiedler2::baseArchivItem_Sound_XMidi, public glArchivItem_Music
{
public:
	/// Konstruktor von @p glArchivItem_Sound_XMidi.
	glArchivItem_Sound_XMidi(void) : baseArchivItem_Sound(), baseArchivItem_Sound_XMidi(), glArchivItem_Music() {}

	/// Kopierkonstruktor von @p glArchivItem_Sound_XMidi.
	glArchivItem_Sound_XMidi(const glArchivItem_Sound_XMidi *item) : baseArchivItem_Sound(item), baseArchivItem_Sound_XMidi(item), glArchivItem_Music(item) {}

	/// Spielt die Musik ab.
	void Play(const unsigned repeats);
};

#endif // !GLARCHIVITEM_SOUND_XMIDI_H_INCLUDED
