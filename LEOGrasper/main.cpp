/*******************************************************************
 ** Copyright (C) 2012-2013 Mikhail Frank, Leo Pape, Juxi Leitner **
 ** CopyPolicy: Released under the terms of the GNU GPL v2.0.     **
 *******************************************************************/

#include "hand_controller.h"
#include <string>
#include <iostream>

#define VOCAB_PREGRASP VOCAB3('p','r','e')	// goto pregrasp pose  (position control)
#define VOCAB_GRASP VOCAB3('c','l','s')		// close the hand      (velocity control)
#define VOCAB_UNGRASP VOCAB3('o','p','n')	// open the hand       (velocity control)
#define VOCAB_HELP VOCAB4('h','e','l','p')	// display help
#define VOCAB_TRAJ VOCAB4('t','r','a','j')	// set new trajectory
#define VOCAB_GET VOCAB3('g','e','t')	    // get the current trajectory
#define VOCAB_SET VOCAB3('s','e','t')		// load trajectory from file
#define VOCAB_LOAD VOCAB4('l','o','a','d')	// reload trajectories from configuration file

int main(int argc, char *argv[]) {
	Network yarp;
	Property command;
	command.fromCommand(argc, argv);
    
    
    // check for configuration file
    Property file;
	Bottle settings;
	bool hasConfigFile = false;
	std::string fname;
	if (command.check("file")) {
		fname = command.find("file").asString();
        std::cout << "\nLoading Configuration File: " << fname << std::endl;
        hasConfigFile = file.fromConfigFile(fname.c_str());
        
        // check if filename is actually a file, warn if not!
        if(!hasConfigFile) std::cout << "\nWARNING! Configuration file not found!!" << std::endl;
		settings = file.findGroup("SETTINGS");
	}
    

	// the following values are read from the configuration file, but can be overwritten from the command line
	std::string robot = "icubSim"; // default is simulator
	if ( settings.check("robot") )  { robot = settings.find("robot").asString().c_str(); }
	if ( command.check("robot") )  { robot = command.find("robot").asString().c_str(); }

	std::string part = "right_arm";	// whether or not to use the left hand (default is right)
	if ( settings.check("part") )  { part = settings.find("part").asString().c_str(); }
	if ( command.check("part") )  { part = command.find("part").asString().c_str(); }

	std::string portname = "/" + robot + "/" + part + "/" + "grasp";	// the name of the RPC server
	if ( settings.check("port") )  { portname = settings.find("port").asString().c_str(); } 
	if ( command.check("port") )  { portname = command.find("port").asString().c_str(); } 

	double errtol = 0.4;	// 0-1, where smaller numbers favor precision
	if ( settings.check("tolerance") )  { errtol = settings.find("tolerance").asDouble(); }
	if ( command.check("tolerance") )  { errtol = command.find("tolerance").asDouble(); }

	double attFactor = 4.0;	// 0-inf larger numbers cause the controller to approach target points more slowly
	if ( settings.check("attenuation") )  { attFactor = settings.find("attenuation").asDouble(); }
	if ( command.check("attenuation") )  { attFactor = command.find("attenuation").asDouble(); }

	double refAccel = 10.0;	// reference acceleration for the yarp remote control board interface
	if ( settings.check("acceleration") )  { refAccel = settings.find("acceleration").asDouble(); }
	if ( command.check("acceleration") )  { refAccel = command.find("acceleration").asDouble(); }

	double refSpd = 20.0;	// reference velocity for position control moves
	if ( settings.check("refspeed") )  { refSpd = settings.find("refspeed").asDouble(); }
	if ( command.check("refspeed") )  { refSpd = command.find("refspeed").asDouble(); }

	double movSpd = 70.0;	// target velocity during velocity control moves
	if ( settings.check("velocity") )  { movSpd = settings.find("velocity").asDouble(); }
	if ( command.check("velocity") )  { movSpd = command.find("velocity").asDouble(); }

	double pidMaxOut = 400.0;	// maximum PID output
	if ( settings.check("pidmaxout") )  { pidMaxOut = settings.find("pidmaxout").asDouble(); }
	if ( command.check("pidmaxout") )  { pidMaxOut = command.find("pidmaxout").asDouble(); }
	pidMaxOut = min(MAX_PID_OUT, pidMaxOut);

	double duration = 6.0;	// default duration
	if ( settings.check("duration") )  { duration = settings.find("duration").asDouble(); }
	if ( command.check("duration") )  { duration = command.find("duration").asDouble(); }

	double waitStuck = 2.0;	// default waiting time for stuck joints
	if ( settings.check("waitstuckjoint") )  { waitStuck = settings.find("waitstuckjoint").asDouble(); }
	if ( command.check("waitstuckjoint") )  { waitStuck = command.find("waitstuckjoint").asDouble(); }
    
	// Open controller
	HandController controller( errtol, attFactor, refAccel, refSpd, waitStuck );
	
	// Load trajectory from file
	vector<ControlPoint> graspTrajectory;
	Bottle group;
	bool success = false;
	if (hasConfigFile) {
		if (part.compare("left_arm") == 0) {
			group = file.findGroup("LEFT_DEFAULT");
			success = controller.bottle2Trajectory(group, graspTrajectory);
		} else if (part.compare("right_arm") == 0) {
			group = file.findGroup("RIGHT_DEFAULT");
			success = controller.bottle2Trajectory(group, graspTrajectory);
		}
		if (success) std::cout << "Loaded default grasp trajectory for " << part << std::endl;
		else {
            std::cout << "WARNING! Could not load default grasp trajectory from file, loading default!" << std::endl;
            
            // if trajectory could not be loaded from file, load default trajectory
            controller.defaultGrasp(graspTrajectory, part);
        }
            
	}
    
    // Finished reading configuration file
    std::cout << "Setting up, done!" << std::endl << std::endl;
	    
    // Initialize the controller
	if (!controller.init(robot.c_str(), part.c_str())) { return 0; }

	// load PID settings to robot
	controller.setPIDs(settings, part, pidMaxOut);

	// open rpc port
	yarp::os::RpcServer port;
	port.open(portname.c_str());

	int thisVel;
	double thisDur, thisPidMax;

	while (true) {
		cout << "Waiting for a message..." << endl;
		yarp::os::Bottle query;
		yarp::os::Bottle response;
		port.read(query,true);

		int command = query.get(0).asVocab();

		switch ( command ) {
		case VOCAB_PREGRASP:
			thisDur = query.get(1).asDouble();
			if ( thisDur <= 0.0 ) { thisDur = duration; }

			if (controller.preGrasp(graspTrajectory[0], thisDur)) {
				response.addString("OK");
			} else {
				response.addString("FAIL");
			}
			break;
		case VOCAB_GRASP:
			if (query.size() <= 1) {thisVel = movSpd; }
			else {thisVel = query.get(1).asInt(); }
			if (query.size() <= 2) {thisDur = duration; }
			else { thisDur = query.get(2).asDouble(); }
			if (query.size() <= 3) {thisPidMax = pidMaxOut; }
			else { thisPidMax = min(MAX_PID_OUT, query.get(3).asDouble() * MAX_PID_OUT); }
			
			if (controller.grasp( graspTrajectory, thisVel, thisDur, thisPidMax )) {
				response.addString("OK");
			} else {
				response.addString("FAIL");
			}
			break;
		case VOCAB_UNGRASP:
			if (query.size() <= 1) {thisVel = movSpd; }
			else {thisVel = query.get(1).asInt(); }
			if (query.size() <= 2) {thisDur = duration; }
			else { thisDur = query.get(2).asDouble(); }
			if (query.size() <= 3) {thisPidMax = pidMaxOut; }
			else { thisPidMax = min(MAX_PID_OUT, query.get(3).asDouble() * MAX_PID_OUT); }

			if (controller.unGrasp( graspTrajectory, thisVel, thisDur, thisPidMax )) {
				response.addString("OK");
			} else {
				response.addString("FAIL");
			}
			break;
		case VOCAB_TRAJ:
			success = controller.bottle2Trajectory(query, graspTrajectory);
			if (success) {
				cout << "New trajectory successfully set." << endl;
				response.addString("OK");
			} else {
				cout << "Trajectory setting failed." << endl;
				response.addString("FAIL");
		    }
			break;
		case VOCAB_GET:
			for (int iPose = 0; iPose<graspTrajectory.size(); iPose++) {
				yarp::os::Bottle pose;
				ControlPoint cpoint = graspTrajectory[iPose];
				for (int iJoint = 0; iJoint<NFJOINTS; iJoint++){
					pose.addDouble(cpoint.p[iJoint]);
				}
				response.addList() = pose;
			}
			break;
		case VOCAB_SET:
			success = (query.size() > 1) && query.get(1).isString() && hasConfigFile;
			if (success) {
				group = file.findGroup(query.get(1).asString());
				success = controller.bottle2Trajectory(group, graspTrajectory);
			}
			if (success) {
				cout << "Set trajectory to " << query.get(1).asString() << " in configuration file " << endl;
				response.addString("OK");
			} else {
				cout << "Set trajectory failed." << endl;
				response.addString("FAIL");
		    }
			break;
		case VOCAB_LOAD:
			// reload configuration file
			hasConfigFile = file.fromConfigFile(fname.c_str());
			success = (query.size() > 1) && query.get(1).isString() && hasConfigFile;
			if (success) {
				group = file.findGroup(query.get(1).asString());
				success = controller.bottle2Trajectory(group, graspTrajectory);
			}
			if (success) {
				cout << "Trajectory loaded from " << query.get(1).asString() << " in " << fname << endl;
				response.addString("OK");
			} else {
				cout << "Trajectory loadig failed." << endl;
				response.addString("FAIL");
		    }
			break;
		case VOCAB_HELP:
			response.addVocab(Vocab::encode("many"));
			response.addString("Grasp Controller: \n");
			response.addString("pre [duration]: perform pregrasp for DURATION seconds.\n");
			response.addString("cls [speed] [duration] [maxpid]: close with SPEED (angles per \n    second) for DURATION seconds, with proportion MAXPID (0 - 1)\n    squeezing force.\n");
			response.addString("opn [speed] [duration] [maxpid]: open with SPEED (angles per \n    second) for DURATION seconds, with proportion MAXPID (0 - 1)\n    opening force.\n");
			response.addString("traj (p_11, ..., p_19), ..., (p_n1, ..., p_n9): \n    set trajectory points 1-n with position of hand\n    joints 1-9 in degrees.\n");
			response.addString("load NAME: reload configuration file and set trajectory to section NAME. \n    Configuration file location should be specified as command\n    line argument during module startup.\n");
			response.addString("get: return current trajectory.\n");
			response.addString("set NAME: set trajectory to section NAME from previously loaded \n    configuration file. Configuration file location should be specified\n    as command line argument during module startup.\n");
			break;
		default:
			response.addString("Unknown Command. Type help for more information.");
			break;
		}

		port.reply(response);
	}
	return 0;
}
