// $Id: iwStatistics.h 5855 2010-01-04 16:44:38Z FloSoft $
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
#ifndef iwSTATISTICS_H_INCLUDED
#define iwSTATISTICS_H_INCLUDED

#pragma once

#include "IngameWindow.h"

/// Fenster mit den Statistiken.
class iwStatistics : public IngameWindow
{

public:

	/// Konstruktor von @p iwStatistics.
	iwStatistics();
	~iwStatistics();
  
private:

  StatisticType currentView;
  StatisticTime currentTime;
  ctrlText *headline;
  ctrlText *maxValue;
  ctrlText *minValue;
  std::vector<ctrlText*> timeAnnotations;
  std::vector<bool> activePlayers;
  unsigned numPlayingPlayers;

  void Msg_ButtonClick(const unsigned int ctrl_id);
  void Msg_PaintAfter();
  void Msg_OptionGroupChange(const unsigned int ctrl_id, const unsigned short selection);
  void DrawStatistic(StatisticType type);
  void DrawAxis();
};

#endif // !iwSTATISTICS_H_INCLUDED
