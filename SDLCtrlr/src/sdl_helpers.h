// Copyright: (C) 2014 Juxi Leitner
// Author: Juxi Leitner <juxi.leitner@gmail.com>
// find more information at http://Juxi.net/
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef SDL_HELPERS_H
#define SDL_HELPERS_H

#include "SDL/SDL.h"
#include "SDL/SDL_image.h"
#include "dot.h"
#include <string>
#include <vector>

#define SCREEN_WIDTH    640
#define SCREEN_HEIGHT   480
#define SCREEN_BPP      32
//Screen attributes

#define MIN_CONTROLTIME 25  // 25 ms don't be quicker!

extern SDL_Surface *screen;
extern SDL_Joystick *joystick;

bool sdl_init(const char *title);
void sdl_cleanup();

void sdl_apply_surface(int x, int y, SDL_Surface* source, SDL_Surface* destination, SDL_Rect* clip = NULL );
void sdl_update_screen();

void sdl_add_object(class Dot *);

SDL_Surface* sdl_load_image( std::string filename );

#endif