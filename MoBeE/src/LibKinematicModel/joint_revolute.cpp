/*******************************************************************
 ***               Copyright (C) 2011 Mikhail Frank              ***
 ***  CopyPolicy: Released under the terms of the GNU GPL v2.0.  ***
 ******************************************************************/

#include "joint_revolute.h"


using namespace KinematicModel;

RevoluteJoint::RevoluteJoint( Robot* robot, int part, KinTreeNode* parent, Motor* motor ) :
                        Joint( robot, part, parent, motor, RJOINT )
{	
    limits = Interval();
	limits.setMin(-M_PI);
	limits.setMax(M_PI);
}

void RevoluteJoint::setM()
{
	M.setAxisAngleOrientation(nodeAxis, position);
}
