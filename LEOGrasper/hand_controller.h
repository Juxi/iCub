/*******************************************************************
 ** Copyright (C) 2012-2013 Mikhail Frank, Leo Pape, Juxi Leitner **
 ** CopyPolicy: Released under the terms of the GNU GPL v2.0.     **
 *******************************************************************/

#ifndef HAND_CONTROLLER_H
#define HAND_CONTROLLER_H

#include <vector>
#include <memory>
#include <string>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

//#define DEBUG
#define NAJOINTS 16
#define NFJOINTS 9
#define JOINTOFFSET 7
#define MAX_PID_OUT 1333.
using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

struct ControlPoint {
	double t;
	double p[NFJOINTS];
};

static ControlPoint currentPoint(yarp::dev::IEncoders *enc) {
	ControlPoint spoint;        // starting point
	double encpos[NAJOINTS];    //encoder positions
	enc->getEncoders(encpos);
	memcpy(&spoint.p[0], &encpos[JOINTOFFSET], NFJOINTS*sizeof(double));	
	return spoint;
};


static ControlPoint operator- (const ControlPoint &a, const ControlPoint &b) { 
	ControlPoint c; 
	for (int i = 0; i<NFJOINTS; i++) {c.p[i] = a.p[i] - b.p[i]; }
	return c;
}

static ControlPoint operator+ (const ControlPoint &a, double b) { 
	ControlPoint c; 
	for (int i = 0; i<NFJOINTS; i++) {c.p[i] = a.p[i] + b; }
	return c;
}

class HandController {
	
public:
	HandController( double errorTolerance, double attenuationFactor, int refAcceleration, int refSpeed, double waitStuck );
    ~HandController();
	
	bool init(const char *robotName, const char *partName);		//!< Connects to the remote device
	void close();												//!< Closes the connection to the remote device
	bool checkValidity();										//!< Checks if the remote device is ready and the interface is working
	
	void defaultGrasp(vector<ControlPoint> &graspTrajectory, std::string part);	//!< Writes default grasp
	void array2Trajectory(double* graspArray, int npoints, vector<ControlPoint> &graspTrajectory);
	bool bottle2Trajectory(Bottle& b, vector<ControlPoint> &graspTrajectory);

	bool preGrasp(const ControlPoint &preGrasp, double duration);
	bool grasp(const vector<ControlPoint> &trajectory, int speed, double duration, double maxpidout);
	bool unGrasp(const vector<ControlPoint> &trajectory, int speed, double duration, double maxpidout);

	void setPIDs(Bottle &settings, std::string part, double maxpidout);

	/* need a function to make the trajectory from a config file */
	
private:
	double errorTolerance;		    //!< 0-1, where smaller numbers favor precision
	double attenuationFactor;		//!< 0-inf larger numbers cause the controller to approach target points more slowly
	double waitStuckJoint;          //!< number of seconds to wait before stopping joints that are stuck
	double delay;					//!< delay in waiting loops in millis
	double prvabe[NFJOINTS];        //!< previous absolute error (for masking stuck joints)
	double scalepwm[NFJOINTS];      //!< PWM outputs are scaled for each joint individually
	double refSpeed, refAcceleration, lastMaxPIDout;

	Network network;
	PolyDriver *dd;
	IPositionControl *pos;
	IVelocityControl *vel;
	IEncoders *enc;
	IPidControl *pid;
	IAmplifierControl *amp;
	IControlLimits *lim;
	IOpenLoopControl *opn;
		
	struct Interval {
		double min;
		double max;
	};

	Interval limits[NFJOINTS];
	
	void setMaxPIDout(double maxpidout);
	void stopHand();
	void resetPrevValues();
	bool blockingPositionMove(const ControlPoint &target, double duration);
	double doVelocityControl(int speed, const ControlPoint &cpoint, const ControlPoint &direction, vector<bool>& mask, bool stopOnTouch);
	
};
#endif
/** @} */
