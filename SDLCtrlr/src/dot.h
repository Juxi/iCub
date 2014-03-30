// Copyright: (C) 2014 Juxi Leitner
// Author: Juxi Leitner <juxi.leitner@gmail.com>
// find more information at http://Juxi.net/
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef DOT_HELPER_H
#define DOT_HELPER_H

#include <string>
#include "yarp_helpers.h"
#include "sdl_helpers.h"

//The dot dimensions
#define DOT_WIDTH  20
#define DOT_HEIGHT 20

class Dot {
    private:
        // The position of the dot
        int x, y;
        int centerX, centerY;
        // The velocities of the dot
        int xVel, yVel, zVel;
        
        // 
        bool grasp_started;
        
        
        int axis_offset;
        
    public:
        SDL_Surface *surface;
        
        // Constructor
        Dot();
        Dot(int in_axis, std::string file);        

        // handle and parse the event input
        void handle_input(const SDL_Event& event);
        void handle_sixaxis(const SDL_Event& event);

        // move the object
        void move();
        void center();

        void move_yarp();
        
        // raw the dot
        void draw();
};

#endif