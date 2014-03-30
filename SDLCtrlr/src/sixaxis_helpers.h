// Copyright: (C) 2014 Juxi Leitner
// Author: Juxi Leitner <juxi.leitner@gmail.com>
// find more information at http://Juxi.net/
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef SIXAXIS_HELPERS_H
#define SIXAXIS_HELPERS_H

#include "sdl_helpers.h"

enum EPS3RawButtonIndex
{
    EPS3RawButtonIndex_Select           = 0,

    EPS3RawButtonIndex_JoyClickLeft     = 1,
    EPS3RawButtonIndex_JoyClickRight    = 2,

    EPS3RawButtonIndex_Start            = 3,

    EPS3RawButtonIndex_DpadUp           = 4,
    EPS3RawButtonIndex_DpadRight        = 5,
    EPS3RawButtonIndex_DpadDown         = 6,
    EPS3RawButtonIndex_DpadLeft         = 7,

    EPS3RawButtonIndex_TriggerLeft      = 8,
    EPS3RawButtonIndex_TriggerRight     = 9,

    EPS3RawButtonIndex_ShoulderLeft     = 10,
    EPS3RawButtonIndex_ShoulderRight    = 11,

    EPS3RawButtonIndex_Triangle         = 12,
    EPS3RawButtonIndex_Circle           = 13,
    EPS3RawButtonIndex_Cross            = 14,
    EPS3RawButtonIndex_Square           = 15,

    EPS3RawButtonIndex_PSButton         = 16
};


void sixaxis_handle(const SDL_Event &event);

#endif