// Copyright: (C) 2014 Juxi Leitner
// Author: Juxi Leitner <juxi.leitner@gmail.com>
// find more information at http://Juxi.net/
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef YARP_HELPERS_H
#define YARP_HELPERS_H

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

bool yarp_init(yarp::os::Property &cmd);
void yarp_cleanup();

bool yarp_send_force(int arm_offset, int lr, int ud, int fb);
bool yarp_send_grasp(int arm_offset, bool open);

#endif