/*******************************************************************
 ** Copyright (C) 2012-2013 Mikhail Frank, Leo Pape, Juxi Leitner **
 ** CopyPolicy: Released under the terms of the GNU GPL v2.0.     **
 *******************************************************************/

#include "hand_controller.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <cmath>
#include <algorithm>


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

HandController::HandController( double errTol, double attFactor, int refAccel, int refSpd, double waitStuck ) : 
		errorTolerance(errTol), attenuationFactor(attFactor), refAcceleration(refAccel), refSpeed(refSpd), waitStuckJoint(waitStuck),
		dd(NULL), pos(NULL), vel(NULL), enc(NULL), pid(NULL), amp(NULL), lim(NULL) {

	delay = 20; // delay in millis
	lastMaxPIDout = 0.;
	resetPrevValues();
	for (int i = 0; i<NFJOINTS; i++ ) { scalepwm[i] = 1.; }
}

HandController::~HandController() {
	close();
}


void HandController::defaultGrasp(vector<ControlPoint> &graspTrajectory, std::string part) {
	// default grasping poses for iCubLugano01, from opened to closed
	
	if (part.compare("left_arm") == 0) {
	// joint num		    			7    8    9   10  11  12  13   14   15
		int nPoints = 5;
		double grasp[5][NFJOINTS] = { { 5,  5,   15,   5, 10, 10, 10,  10,  35 },
									 {  5,  55,  15,   5, 10, 10, 10,  10,  35 },
									 {  5,  55,  15,  20, 35, 20, 30,  30,  90 },
									 {  5,  55,  15,  70, 40, 45, 40, 100, 100 },
									 {  5,  55,  15, 105, 80, 65, 60, 130, 180 }};

		array2Trajectory(&grasp[0][0], nPoints, graspTrajectory);
	} else if (part.compare("right_arm") == 0) {
	// joint num		    			 7    8    9   10  11   12  13   14   15
		int nPoints = 5;
		double grasp[5][NFJOINTS] = { {  25,   5,  15,  10, 10,  10, 20,  10,  15 },
							 		 {  25,  45,  15,  10, 10,  10, 20,  10,  15 },
									 {  25,  60,  15,  30, 35,  25, 40,  30,  60 },
									 {  25,  60,  15,  65, 40,  70, 55, 100,  90 },
									 {  25,  60,  15, 100, 50, 100, 60, 130, 120 }};

		array2Trajectory(&grasp[0][0], nPoints, graspTrajectory);
	} else {
		throw "Part should be either left_arm or right_arm";
	}
}

void HandController::array2Trajectory(double* graspArray, int nPoints, vector<ControlPoint> &graspTrajectory) {
	graspTrajectory.clear();

	for (int i = 0; i < nPoints; i++) {
		ControlPoint cpoint;
		cpoint.t = 0;
		memcpy(&cpoint.p[0], &graspArray[i*NFJOINTS], NFJOINTS*sizeof(double));
		graspTrajectory.push_back(cpoint);
	}
}

bool HandController::bottle2Trajectory(Bottle& cmd, vector<ControlPoint> &graspTrajectory) {
	// go through cmd until the first list of length NFJOINTS is found
	bool success = false;
	int offset = 0;
	for (int i=0; i<cmd.size(); i++) {
		if (cmd.get(i).isList() && !cmd.get(i).isString() && (cmd.get(i).asList()->size() == NFJOINTS)) {
			success = true;
			offset = i;
			break;
		}
	}
	if (!success)
		return false;
	
	// if a valid entry was found, go through the list of control points, and copy them to the trajectory
	vector<ControlPoint> newTraj;
	success = (cmd.size() - offset) > 0;
	for (int iPose = offset; iPose<cmd.size(); iPose++) {
		if (cmd.get(iPose).isList() && cmd.get(iPose).asList()->size() == NFJOINTS) {
			yarp::os::Bottle* pose = cmd.get(iPose).asList();
			ControlPoint cpoint;
			for (int iJoint = 0; iJoint<pose->size(); iJoint++){
				cpoint.p[iJoint] = pose->get(iJoint).asDouble();
			}
			newTraj.push_back(cpoint);
		} else { 
			success = false; break; 
		}
	}
	if (success) {
		graspTrajectory = newTraj; // '=' on vectors is copy operation
	}
	return success;
}


bool HandController::init(const char *robot, const char *part ) {
	if ( !network.checkNetwork() ) { throw "Cannot find YARP network."; }
	
	string robotName = robot;
	string partName = part;
		
	Property options;
	options.put( "robot", robotName.c_str() ); // typically from the command line.
	options.put( "device", "remote_controlboard" );
	
	string localPort = "/" + robotName + "/" + partName + "/hand_control";
	options.put("local", localPort.c_str());
	
	string remotePort = "/" + robotName + "/" + partName;
	options.put("remote", remotePort.c_str());
	
	dd = new PolyDriver(options);
	
	if ( dd ) {
		dd->view(pos);
		dd->view(opn);
		dd->view(vel);
		dd->view(enc);
		dd->view(pid);
		dd->view(amp);
		dd->view(lim);
	}
	
	if (!checkValidity()) { return false; }
	
	int numJoints;
	pos->getAxes(&numJoints);
	if ( numJoints != NAJOINTS ) {
		std::ostringstream oss;
		oss << "Wrong number of DOFs: HandController must connect to an arm of the iCub robot, which has " << NFJOINTS << " joints. The requested device " <<
			remotePort << " has " << numJoints << " joints.";
		throw oss.str();
	}
	
	// enable amplifiers and pid controllers
	for (int i=0; i<NFJOINTS; i++) {
		pos->setRefAcceleration(i+JOINTOFFSET, refAcceleration);
		pos->setRefSpeed(i+JOINTOFFSET, refSpeed );
		lim->getLimits(i+JOINTOFFSET, &(limits[i].min), &(limits[i].max));
		amp->enableAmp(i+JOINTOFFSET);
		pid->enablePid(i+JOINTOFFSET);
	}

#ifdef USE_TOUCH
	// init alex's touch stuff
	string remoteTouchPort;
	string touchPartName;
	string strippedRobotName;
	if ( robotName == "icubSim" || robotName == "icubSimF" ) {
		if ( partName == "left_arm" ) touchPartName = "left_hand";
		else if ( partName == "right_arm" ) touchPartName = "right_hand";
		else throw "Not connected to an arm";
		strippedRobotName = "icubSim";
	} else if ( robotName == "icub" || robotName == "icubF" ) {
		if ( partName == "left_arm" ) touchPartName = "lefthand";
		else if ( partName == "right_arm" ) touchPartName = "righthand";
		else throw "Not connected to an arm";
		strippedRobotName = "icub";
	}
	else throw "Not connected to an iCub";
	
	remoteTouchPort = "/";
	remoteTouchPort.append(strippedRobotName.c_str());
	remoteTouchPort += "/skin/" + touchPartName;

	ts = new TouchSenseThread( remoteTouchPort, "/touchsensor" );
	
	if (!ts->start()) {
		throw "Cannot start touch sensor thread";
	};

#endif

	cout << "HandController started successfully." << endl;
	return true;
}


bool HandController::checkValidity() {
	if ( !dd ) { cout << "Error: PolyDriver was not created." << endl; return false; }
	else if ( !dd->isValid() ) { cout << "Error: PolyDriver is invalid." << endl; return false; }
	if (!pos) { cout << "IPositionControl error" << endl; return false; }
	if (!opn) { cout << "IOpenLoopControl error" << endl; return false; }
	if (!vel) { cout << "IVelocityControl error" << endl; return false; }
	if (!enc) { cout << "IEncoders error." << endl; return false; }
	if (!pid) { cout << "IPidControl error." << endl; return false; }
	if (!amp) { cout << "IAmplifierControl error." << endl; return false; }
	if (!lim) { cout << "IControlLimits error." << endl; return false; }
	return true;
}


void HandController::close() {
#ifdef USE_TOUCH
	ts->stop();
#endif
	
	if (vel || pos) { stopHand(); }

	for (int i=0; i<NFJOINTS; i++) {
	    if (amp) { amp->disableAmp(i+JOINTOFFSET); }
	    if (pid) { pid->disablePid(i+JOINTOFFSET); }
	}

	if (dd) { 
		dd->close();
		delete dd;
	}
}


bool HandController::preGrasp(const ControlPoint &preGrasp, double duration) {
	if (!checkValidity()) { return false; }
	return blockingPositionMove(preGrasp, duration);
}


bool HandController::blockingPositionMove(const ControlPoint &target, double duration) {
	for (int i = 0; i<NFJOINTS; i++ )
		pos->positionMove( i+JOINTOFFSET, target.p[i] );
	
	if (duration == 0)
		return true;

	bool finished = false;
	double start = Time::now();

	while ( !finished ) {
		pos->checkMotionDone(&finished);
		Time::delay(delay/1000.0); // we don't need precise timing here
		if ((Time::now() - start) > duration) {
			cout << "Warning: timeout while waiting for reaching position." << endl;
			return false;
		}
	}
	
	cout << "Pose reached, error: ";
	double encpos[NAJOINTS];
	enc->getEncoders(encpos);
	for (int i=0; i<NFJOINTS; i++ ) {
		cout << target.p[i] - encpos[i+JOINTOFFSET] << "\t";
	}
	cout << endl;
	return true;
}


void HandController::setPIDs(Bottle &settings, std::string part, double maxpidout) {
	yarp::dev::Pid thispid;

	std::string partString;
	if (part.compare("left_arm") == 0) {
		partString = "left";
	} else if (part.compare("right_arm") == 0) {
		partString = "right";
	} else {
		throw "Part should be either left_arm or right_arm";
	}
	

	std::string str;
	cout << "Setting PID controller values for " << partString << " hand fingers." << endl;
	for (int i=0; i<NFJOINTS; i++) {
		// The PID's integrator needs to be reset before increasing the maxpidout, otherwise it will send a jolt through the motors
		// Unfortunately, just resetting the PIDs with pid->resetPid doesn't do anything :(.
		pid->disablePid(i+JOINTOFFSET);
		pid->enablePid(i+JOINTOFFSET);
		pid->getPid(i+JOINTOFFSET, &thispid);

		// note thate elements in settings.findGroup(...) are shifted by 1, because the variable name is also in the list
		str = partString + "kp";
		if (settings.check(str.c_str()) && settings.findGroup(str.c_str()).size() == (NFJOINTS + 1))
			thispid.setKp(settings.findGroup(str.c_str()).get(i+1).asDouble());
		str = partString + "ki";
		if (settings.check(str.c_str()) && settings.findGroup(str.c_str()).size() == (NFJOINTS + 1))
			thispid.setKi(settings.findGroup(str.c_str()).get(i+1).asDouble());
		str = partString + "kd";
		if (settings.check(str.c_str()) && settings.findGroup(str.c_str()).size() == (NFJOINTS + 1))
			thispid.setKd(settings.findGroup(str.c_str()).get(i+1).asDouble());
		str = partString + "scale";
		if (settings.check(str.c_str()) && settings.findGroup(str.c_str()).size() == (NFJOINTS + 1))
			thispid.setScale(settings.findGroup(str.c_str()).get(i+1).asDouble());
		str = partString + "offset";
		if (settings.check(str.c_str()) && settings.findGroup(str.c_str()).size() == (NFJOINTS + 1))
			thispid.setOffset(settings.findGroup(str.c_str()).get(i+1).asDouble());
		str = partString + "maxint";
		if (settings.check(str.c_str()) && settings.findGroup(str.c_str()).size() == (NFJOINTS + 1))
			thispid.setMaxInt(settings.findGroup(str.c_str()).get(i+1).asDouble());
		str = partString + "scalemax";
		if (settings.check(str.c_str()) && settings.findGroup(str.c_str()).size() == (NFJOINTS + 1))
			scalepwm[i] = settings.findGroup(str.c_str()).get(i+1).asDouble();
			thispid.setMaxOut( min(MAX_PID_OUT, scalepwm[i]*maxpidout) );

		cout << "kp " << thispid.kp << " ki " << thispid.ki << " kd " << thispid.kd << " scale " << thispid.scale <<
			" maxint " << thispid.max_int << " offset " << thispid.offset << " max_output " << thispid.max_output << endl;
		pid->setPid(i+JOINTOFFSET, thispid);
	}
	lastMaxPIDout = maxpidout;
}

void HandController::setMaxPIDout(double maxpidout) {
	// set maximum PID output for hand joints
	if (lastMaxPIDout == maxpidout) // only write new maxpidout when it changed, because it takes a long time (> 1 second)
		return;

	yarp::dev::Pid thispid;

	for (int i=0; i<NFJOINTS; i++) {
		// The PID's integrator needs to be reset before increasing the maxpidout, otherwise it will send a jolt through the motors
		// Unfortunately, just resetting the PIDs with pid->resetPid doesn't do anything :(.
		pid->disablePid(i+JOINTOFFSET);
		pid->enablePid(i+JOINTOFFSET);
		pid->getPid(i+JOINTOFFSET, &thispid);
		thispid.setMaxOut( min(MAX_PID_OUT, scalepwm[i]*maxpidout) );
		pid->setPid(i+JOINTOFFSET, thispid);
	}
	lastMaxPIDout = maxpidout;

#ifdef DEBUG
	Pid p[NAJOINTS];
	pid->getPids(p);
	cout << "setting new PID maxima:";
	for ( int i = 0; i<NAJOINTS; i++ ) {
		cout << " " << p[i].max_output;
	}
	cout << endl;	
#endif
}

void HandController::stopHand() {
	resetPrevValues();

	// Stop the fingers using position control command!
	// This should NOT be done using a velocity control command, because when a finger get loose after being stuck, bringing its velocity to 0 can leave it at a weird position.
	cout << "STOP" << endl;
	double currentPos;
	for (int i=0; i<NFJOINTS; i++ ) {
		// unfortunately, resetting the PIDs with pid->resetPid doesn't do anything :(, so we do it thusly:
		enc->getEncoder(i+JOINTOFFSET, &currentPos);
		pos->positionMove(i+JOINTOFFSET, currentPos);
	}
}


void HandController::resetPrevValues() {
	for (int i = 0; i<NFJOINTS; i++ ) { 
		//prvvel[i] = 0.;
		prvabe[i] = 0.;
	}
}


bool HandController::grasp(const vector<ControlPoint> &trajectory, int speed, double duration, double maxpidout) {
	double currentAttenuation;
	bool moving;
	int npos = trajectory.size();
	vector<double> failTimes(NFJOINTS, 0.);
	vector<bool> oldMask;
	vector<bool> mask(NFJOINTS, false);
	int count = 0;

	// some error checking
	if (npos <=0) { 
		cout << "Warning: no trajectory points specified." << endl;
		return false; 
	}
	if (!checkValidity()) { return false; }

	ControlPoint direction = trajectory[npos-1] - trajectory[0]; // movement direction
	stopHand();
	setMaxPIDout(maxpidout);
	
	double startTime = Time::now();
	double nowTime, cntTime, waitTime;

	for (int i=0; i<npos; i++) {
		moving = true;
		
		cout << "Targetting control point " << i+1 << " out of " << npos <<  endl;
		while (moving) {
			oldMask = mask; // copy
			currentAttenuation = doVelocityControl(speed, trajectory[i], direction, mask, true);

			// update movement failure mask
			for (int j=0; j<NFJOINTS; j++) {
				if (mask[j]) { failTimes[j] = failTimes[j] + delay/1000.; }
				else {failTimes[j] = 0.;}
				if ((failTimes[j] > waitStuckJoint) && !oldMask[j])
					cout << "Joint " << j+JOINTOFFSET << " is stuck and will not be moved further" << endl;
				mask[j] = failTimes[j] > waitStuckJoint; // after waitStuckJoint seconds failure, the joint will become masked
			}

			// timing
			count++;
			nowTime = Time::now();
			cntTime = startTime + count * delay/1000.; // expected time according to counter
			waitTime = cntTime - nowTime; // if expected time is before counter, wait a little bit
			if (waitTime > 0.)
				Time::delay(waitTime);
			if ((cntTime - startTime) >= duration) {
				cout << "Warning: timeout while waiting for grasp to complete." << endl;
				blockingPositionMove(trajectory[npos-1], 0); // go to final position using position move
				return false;
			}
			if (i < (npos - 1))
				moving = currentAttenuation > errorTolerance;
			else
				moving = currentAttenuation > (errorTolerance/3);
		}
	}
	
	// after unsuccessful grasp go to end position with position move
	blockingPositionMove(trajectory[npos-1], 0);
	return true;
}


bool HandController::unGrasp(const vector<ControlPoint> &trajectory, int speed, double duration, double maxpidout) {
	double currentAttenuation;
	bool moving;
	int npos = trajectory.size();
	vector<bool> mask(NFJOINTS, false);
	vector<bool> oldMask;
	vector<double> failTimes(NFJOINTS, 0.);
	int count = 0;

	// some error checking
	if (npos <=0) { 
		cout << "Warning: no trajectory points specified." << endl;
		return false; 
	}
	if (!checkValidity()) { return false; }

	ControlPoint direction = trajectory[0] - trajectory[npos-1]; // movement direction
	stopHand();
	setMaxPIDout(maxpidout);
	
	double startTime = Time::now();
	double nowTime, cntTime, waitTime;
	for (int i = npos-1; i >= 0; i--) {
		moving = true;

		cout << "Targetting control point " << i+1 << " out of " << npos << endl;
		while (moving) {
			oldMask = mask; // copy
			currentAttenuation = doVelocityControl(speed, trajectory[i], direction, mask, true);
			
			// update movement failure mask
			for (int j=0; j<NFJOINTS; j++) { 
				if (mask[j]) { failTimes[j] = failTimes[j] + delay/1000.0; }
				else {failTimes[j] = 0.;}
				if ((failTimes[j] > waitStuckJoint) && !oldMask[j])
					cout << "Joint " << j+JOINTOFFSET << " is stuck and will not be moved further" << endl;
				mask[j] = failTimes[j] > waitStuckJoint; // after waitStuckJoint seconds failure, the joint will become masked
			}
			
			// timing
			count++;
			nowTime = Time::now();
			cntTime = startTime + count * delay/1000.; // expected time according to counter
			waitTime = cntTime - nowTime; // if expected time is before counter, wait a little bit
			if (waitTime > 0.)
				Time::delay(waitTime);
			if ((cntTime - startTime) >= duration) {
				cout << "Warning: timeout while waiting for ungrasp to complete." << endl;
				blockingPositionMove(trajectory[0], 0);
				return false;
			}
			if (i > 0)
				moving = currentAttenuation > errorTolerance;
			else
				moving = currentAttenuation > (errorTolerance/3);
		}
	}
	
	// after unsuccessful grasp go to start position with position move
	blockingPositionMove(trajectory[0], 0);
	return true;
}


double HandController::doVelocityControl( int speed, const ControlPoint &cpoint, const ControlPoint &direction, vector<bool>& mask, bool stopOnTouch ) {
	double encpos[NAJOINTS]; //encoder positions
	double poserr[NFJOINTS]; // position error
	double abserr[NFJOINTS]; // absolute value of error
	double errchn[NFJOINTS]; // error change

	// get encoder positions and tabulate error
	enc->getEncoders(encpos);
	double maxerr = 0.;
	double maxatt = 0.;
	for ( int i=0; i<NFJOINTS; i++ ) {
		poserr[i] = cpoint.p[i] - encpos[i+JOINTOFFSET];
		abserr[i] = abs( poserr[i] );

		errchn[i] = abserr[i] - prvabe[i];
		prvabe[i] = abserr[i];

		// prevent moving in the wrong direction
		if ( (poserr[i] < 0. && direction.p[i] > 0.) || (poserr[i] > 0. && direction.p[i] < 0.) ) {
			poserr[i] = 0.; abserr[i] = 0.;
		}

		// set masked errors to 0
		if (mask[i]) {
			poserr[i] = 0.; abserr[i] = 0.;
		}
		
		maxerr = max(maxerr, abserr[i]);
	}
	if ( maxerr < 1e-6) { return 0.; }

	
	// stop digits that have touched something (i.e., set errors to 0)
#ifdef USE_TOUCH	
		// update booleans for touch sensors
		bool palmTouch, thumbTouch, indexTouch, middleTouch, ringTouch, littleTouch, 
		palmTouch   = ts->getPalmTouch();
		thumbTouch  = ts->getThumbTouch(); 
		indexTouch  = ts->getIndexTouch();
		middleTouch = ts->getMiddleTouch();
		ringTouch   = ts->getRingTouch();
		littleTouch = ts->getLittleTouch();

	if ( stopOnTouch ) {
		if ( thumbTouch || indexTouch || middleTouch || ringTouch || littleTouch ) { abserr[7] = 0; }
		if ( thumbTouch ) { abserr[8] = 0; abserr[9] = 0; abserr[10] = 0; }
		if ( indexTouch ) { abserr[11] = 0; abserr[12] = 0; }
		if ( middleTouch ) { abserr[13] = 0; abserr[14] = 0; }
		if ( littleTouch || ringTouch) { abserr[15] = 0; }
	}
#endif

	// do feedback velocity control
	double t, att, newvel;

#ifdef DEBUG
	printf("desired, encoder,   error,  abserr,   maxerr, direction, t,    att,    newvel\n");
#endif

	for ( int i = 0; i<NFJOINTS; i++ ) {
		t      = abserr[i]/attenuationFactor;
		att    = -exp(-t)*(1.+t)+1.;
		newvel = att * speed * poserr[i]/maxerr;
		maxatt = max(maxatt, att);
		
		// damping
		//if (prvvel[i] != 0) { newvel = damping*prvvel[i] + (1-damping)*newvel; }

		// velocities may not be larger than speed
		if (abs(newvel) > speed) {
			if (newvel > 0.) { newvel = speed; } 
			else { newvel = -speed; }
		}

		// If the decrease in absolute error is no more than -1 (degree) and the joint should be moving, (its scaled desired velocity > 0.25), request a mask.
		// Actual masking is done only when masking is requested persistently for WAITSTUKJOINT seconds.
		mask[i] = mask[i] || (errchn[i] > -1. && abs(newvel/speed) > 0.25);


#ifdef DEBUG
		printf("%7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %4.0f, %7.3f, %7.3f, %7.3f\n", cpoint.p[i], encpos[i+JOINTOFFSET], poserr[i], abserr[i], maxerr, direction.p[i], t, att, newvel);
#endif
		vel->velocityMove(i+JOINTOFFSET,newvel);
		//prvvel[i] = newvel;
	}

#ifdef DEBUG
	printf("\n maximum attennuation = %5.3f / %5.3f\n", maxatt, errorTolerance);
#endif
	return maxatt;
}
