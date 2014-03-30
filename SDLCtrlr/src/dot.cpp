// Copyright: (C) 2014 Juxi Leitner
// Author: Juxi Leitner <juxi.leitner@gmail.com>
// find more information at http://Juxi.net/
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include "dot.h"
#include "sixaxis_helpers.h"

#define SCALING_CONST 200

Dot::Dot()
{
    //Initialize the offsets
    x = 0;
    y = 0;

    //Initialize the velocity
    xVel = 0;
    yVel = 0;  
    zVel = 0;  
    
    // first analogue stick
    axis_offset = 0;
    
    // starting postitions for each obj
    centerX = SCREEN_WIDTH  / 3;
    // doube for the second dot ... just to align the centers
    if(axis_offset > 0) centerX *= 2;
    centerX -= DOT_WIDTH / 2;
    centerY = SCREEN_HEIGHT / 2 - DOT_HEIGHT / 2;
    
    //Load the files
    printf("Loading image\n");    
    surface = sdl_load_image( "dot.bmp" );    
    if( surface == NULL )
    {
        printf("Error: could not load file!\n");
        exit(1);
    }
    
    grasp_started = false;
}

Dot::Dot(int in_axis, std::string file)
{
    //Initialize the offsets
    x = 0;
    y = 0;

    //Initialize the velocity
    xVel = 0;
    yVel = 0;
    zVel = 0;
        
    // whatever analogue stick
    axis_offset = in_axis;
    
    // starting postitions for each obj
    centerX = SCREEN_WIDTH  / 3;
    // doube for the second dot ... just to align the centers
    if(axis_offset > 0) centerX *= 2;
    centerX -= DOT_WIDTH / 2;
    centerY = SCREEN_HEIGHT / 2 - DOT_HEIGHT / 2;
    
    
    //Load the files
    printf("Loading image\n");    
    surface = sdl_load_image( file.c_str() );    
    if( surface == NULL )
    {
        printf("Error: could not load file!\n");
        exit(1);
    }
    
    grasp_started = false;
}

void Dot::handle_input(const SDL_Event& event)
{
    //If a axis was changed
    if( event.type == SDL_JOYAXISMOTION )
    {
        //If joystick 0 has moved
        if( event.jaxis.which == 0 )
        {
            //If the X axis changed
            if( event.jaxis.axis == axis_offset )
            {
                //If the X axis is neutral
                if( ( event.jaxis.value > -8000 ) && ( event.jaxis.value < 8000 ) )
                {
                    xVel = 0;
                }
                //If not
                else
                {
                    //Adjust the velocity
                    xVel = event.jaxis.value/SCALING_CONST;
                    
                    // if( event.jaxis.value < 0 )
                    // {
                    //     //xVel = -DOT_WIDTH / 2;
                    //     xVel = events.jaxis.value/DOT_WIDTH;
                    // }
                    // else
                    // {
                    //     xVel = DOT_WIDTH / 2;
                    // }
                }
            }
            //If the Y axis changed
            else if( event.jaxis.axis == axis_offset + 1 )
            {
                //If the Y axis is neutral
                if( ( event.jaxis.value > -8000 ) && ( event.jaxis.value < 8000 ) )
                {
                    yVel = 0;
                }
                //If not
                else
                {
                    yVel = event.jaxis.value/SCALING_CONST;
                    
                    // //Adjust the velocity
                    // if( event.jaxis.value < 0 )
                    // {
                    //     yVel = -DOT_HEIGHT / 2;
                    // }
                    // else
                    // {
                    //     yVel = DOT_HEIGHT / 2;
                    // }
                }
            }
        }
    } else {
        // handle foward/backward
        handle_sixaxis(event);
    }
}


void Dot::handle_sixaxis(const SDL_Event& event) {
    if(axis_offset == 0) {
        switch(event.type) {
            case SDL_JOYBUTTONDOWN:  /* Handle Joystick Button Presses */
                switch( event.jbutton.button ) {
                    case EPS3RawButtonIndex_TriggerLeft:
                        if(! grasp_started) {
                            yarp_send_grasp(axis_offset, false);
                            grasp_started = true;
                        }
                        break;
                    case EPS3RawButtonIndex_ShoulderLeft:
                        if(grasp_started) {
                            grasp_started = false;
                            yarp_send_grasp(axis_offset, true);
                        }
                        break;

                    
                    case EPS3RawButtonIndex_DpadUp:
                        zVel = -600;
                        break;
                    case EPS3RawButtonIndex_DpadDown:
                        zVel = 600;
                        break;
                }
                break;


            // releasing the button press
            case SDL_JOYBUTTONUP:  /* Handle Joystick Button Presses */
                switch( event.jbutton.button ) {                
                    case EPS3RawButtonIndex_DpadDown:
                    case EPS3RawButtonIndex_DpadUp:
                        zVel = 0;
                        break;
                
                }
                break;
            
        }
    } else if(axis_offset == 2) {
        // right
        switch(event.type) {
            case SDL_JOYBUTTONDOWN:  /* Handle Joystick Button Presses */
                switch( event.jbutton.button ) {
                    // right
                    // grasp
                    case EPS3RawButtonIndex_Cross:
                        zVel = 600;
                        break;
                    
                    case EPS3RawButtonIndex_Triangle:
                        zVel = -600;
                        break;

                    case EPS3RawButtonIndex_TriggerRight:
                        yarp_send_grasp(axis_offset, false);
                        grasp_started = true;
                        break;
                    case EPS3RawButtonIndex_ShoulderRight:
                        yarp_send_grasp(axis_offset, true);
                        grasp_started = false;
                        break;

                }
                break;

            // releasing the button press
            case SDL_JOYBUTTONUP:  /* Handle Joystick Button Presses */
                switch( event.jbutton.button ) {                

                    case EPS3RawButtonIndex_Cross:               
                    case EPS3RawButtonIndex_Triangle:  
                        zVel = 0;
                        break;

                }
                break;

        }
    }
    
}


void Dot::center() {
    x = centerX;
    y = centerY;
}

void Dot::move_yarp()
{
    // we updated x and y
    move();
    
    // now do some yarping
    // we need to change the vertical axes here ...
    yarp_send_force(axis_offset, (x - centerX), - (y - centerY), zVel);
}


void Dot::move()
{
    //Move the dot left or right
    x += xVel;

    //If the dot went too far to the left or right
    if( ( x < 0 ) || ( x + DOT_WIDTH > SCREEN_WIDTH ) )
    {
        //move back
        x -= xVel;
    }

    //Move the dot up or down
    y += yVel;

    //If the dot went too far up or down
    if( ( y < 0 ) || ( y + DOT_HEIGHT > SCREEN_HEIGHT ) )
    {
        //move back
        y -= yVel;
    }
}

void Dot::draw()
{
    //Show the dot
    sdl_apply_surface( x, y, surface, screen );
}