// Copyright: (C) 2014 Juxi Leitner
// Author: Juxi Leitner <juxi.leitner@gmail.com>
// find more information at http://Juxi.net/
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include "sdl_helpers.h"


SDL_Surface *screen = NULL;
SDL_Joystick *joystick = NULL;

std::vector<Dot *> obj;

bool sdl_init(const char *title) {
    //Initialize all SDL subsystems
    if( SDL_Init( SDL_INIT_EVERYTHING ) == -1 )
        return false;

    //Set up the screen
    screen = SDL_SetVideoMode( SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP, SDL_SWSURFACE );

    //If there was an error in setting up the screen
    if( screen == NULL )
    {
        return false;
    }

    //Check if there's any joysticks
    if( SDL_NumJoysticks() < 1 )
    {
        return false;
    }

    //Open the joystick
    joystick = SDL_JoystickOpen( 0 );

    //If there's a problem opening the joystick
    if( joystick == NULL )
    {
        return false;
    }

    //Set the window caption
    SDL_WM_SetCaption( title, NULL );
    
    obj.clear();

    //If everything initialized fine
    return true;
}

void sdl_add_object(Dot *o) {
    obj.push_back(o);
}

void sdl_update_screen() {
    
        //Fill the screen white
        SDL_FillRect( screen, &screen->clip_rect, SDL_MapRGB( screen->format, 0xFF, 0xFF, 0xFF ) );

        // // draw all relevant objects
        for(std::vector<Dot*>::iterator it = obj.begin(); it != obj.end(); ++it)
            (*it)->draw();

        //Update the screen
        if( SDL_Flip( screen ) == -1 ) {
            exit(1);
        }
    
}

void sdl_cleanup() {
    //Free the surface
    for(std::vector<Dot*>::iterator it = obj.begin(); it != obj.end(); ++it)
        SDL_FreeSurface((*it)->surface);
    
    //Close the joystick
    SDL_JoystickClose( joystick );
    //Quit SDL
    SDL_Quit();
}


SDL_Surface * sdl_load_image( std::string filename ) {
    //The image that's loaded
    SDL_Surface* loadedImage = NULL;

    //The optimized surface that will be used
    SDL_Surface* optimizedImage = NULL;

    //Load the image
    loadedImage = IMG_Load( filename.c_str() );
    
    //If the image loaded
    if( loadedImage != NULL )
    {
        //Create an optimized surface
        optimizedImage = SDL_DisplayFormat( loadedImage );

        //Free the old surface
        SDL_FreeSurface( loadedImage );

        //If the surface was optimized
        if( optimizedImage != NULL )
        {
            //Color key surface
            SDL_SetColorKey( optimizedImage, SDL_SRCCOLORKEY, SDL_MapRGB( optimizedImage->format, 0, 0xFF, 0xFF ) );
        }
    }
    
    //Return the optimized surface
    return optimizedImage;
}

void sdl_apply_surface( int x, int y, SDL_Surface* source, SDL_Surface* destination, SDL_Rect* clip )
{
    //Holds offsets
    SDL_Rect offset;

    //Get offsets
    offset.x = x;
    offset.y = y;

    //Blit
    SDL_BlitSurface( source, clip, destination, &offset );
}
