// Copyright: (C) 2014 Juxi Leitner
// Author: Juxi Leitner <juxi.leitner@gmail.com>
// find more information at http://Juxi.net/
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include "sixaxis_helpers.h"

void sixaxis_handle(const SDL_Event& event) {
    switch(event.type) {
        case SDL_JOYBUTTONDOWN:  /* Handle Joystick Button Presses */
            printf("joy btn dn\n");        
            if ( event.jbutton.button == EPS3RawButtonIndex_Cross ) {
                printf("we are in cross dwn");
                /* code goes here */
            }
            break;

        case SDL_JOYBUTTONUP:  /* Handle Joystick Button Presses */
                    printf("joy btn up\n");
            if ( event.jbutton.button == EPS3RawButtonIndex_Cross ) {
                /* code goes here */
            }
            break;
        
        case SDL_KEYDOWN:
            /* handle keyboard stuff here */				
            printf("keyboard button down: '%c'", event.key.keysym.unicode);
            break;
        
    }
    
}
