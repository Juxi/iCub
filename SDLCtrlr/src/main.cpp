// Copyright: (C) 2014 Juxi Leitner
// Author: Juxi Leitner <juxi.leitner@gmail.com>
// find more information at http://Juxi.net/
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <yarp/os/all.h>
#include <string>
#include "dot.h"
#include "sdl_helpers.h"
#include "yarp_helpers.h"
#include "sixaxis_helpers.h"

//The event structure
SDL_Event event;


void handle_grasp(const SDL_Event &event);

int main( int argc, char* argv[] )
{
    bool stopped = false;
    const char *title = "SDLController";

    // Initialize SDL and the window
    if( ! sdl_init( title ) )
    {
        printf("Error: could not initialise!\n");
        return 1;
    }
    
    yarp::os::Property command;
    command.fromCommand(argc, argv);

    if( ! yarp_init(command) ) {
        printf("Error: could not initialise YARP!\n");
        return 1;
        
    }    
	

    //Make the dot
    Dot aDot(0, "dota.bmp");
    Dot bDot(2, "dotb.bmp");        
    sdl_add_object(&aDot);
    sdl_add_object(&bDot);    

    // while we are still running
    while( !stopped ) {
        aDot.center();
        bDot.center();
        
        // while there's events to handle
        while( SDL_PollEvent( &event ) ) {
            
            //Handle events for the dot
            aDot.handle_input(event);
            bDot.handle_input(event);

            // the user has closed the window
            if( event.type == SDL_QUIT ) {
                // Stop the program
                stopped = true;
            }
        }
        
        //Move the dots
        aDot.move_yarp();
        bDot.move_yarp();

        sdl_update_screen();
                
        SDL_Delay(MIN_CONTROLTIME);
    }

    //Clean up
    yarp_cleanup();
    sdl_cleanup();

    return 0;
}