/*******************************************************************
 ** Copyright (C) 2013-2014 Juxi Leitner                          **
 ** CopyPolicy: Released under the terms of the GNU GPL v2.0.     **
 *******************************************************************/

#include <string>
#include <iostream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;


#define VOCAB_START VOCAB4('s','t','a','r')
#define VOCAB_STOP  VOCAB4('s','t','o','p')

#define VOCAB_HELP VOCAB4('h','e','l','p')	// display help
#define VOCAB_QUIT VOCAB4('q','u','i','t')


// tell LEOGrasper to open/close hand
bool controlHand(bool open);
bool openHand() { return controlHand(true); }
bool closeHand(){ return controlHand(false); }


std::string robot, part, objName;

int main(int argc, char *argv[]) {
	Network yarp;
	Property command;
	command.fromCommand(argc, argv);
    
	// the following values are read from the configuration file, but can be overwritten from the command line
	robot = "icubSim"; // default is simulator
	if ( command.check("robot") )  { robot = command.find("robot").asString().c_str(); }

	part = "right_arm";	// whether or not to use the left hand (default is right)
	if ( command.check("part") )  { part = command.find("part").asString().c_str(); }

    objName = "cup1"; // the name of object in MoBeE
	if ( command.check("object") )  { objName = command.find("object").asString().c_str(); }

	std::string portName = "/" + robot + "/" + part + "/" + "graspdemo";	// the name of the RPC server
	if ( command.check("port") )  { portName = command.find("port").asString().c_str(); }


    // Finished reading configuration file
    std::cout << "Setting up, done!" << std::endl << std::endl;
	    
	// open rpc port
	yarp::os::RpcServer port;
	port.open(portName.c_str());
    
    bool stopped = false;

	while (!stopped) {
        std::cout << "Waiting for a message..." << std::endl;
        Bottle query;
        Bottle response;
		port.read(query,true);

		int command = query.get(0).asVocab();

		switch ( command ) {
            case VOCAB_START:
                response.addString("OK");
                
                // tell LeoGrasper to opn
                openHand();

                // tell mobee to turn object into "tgt"
//                help.addString("def");
//                help.addString(objectName);
//                help.addString("tgt");
                
                // tell LeoGrasper to cls
                closeHand();
                break;
            case VOCAB_STOP:
                response.addString("OK");

                // tell mobee to turn object into "obs"
//                help.addString("def");
//                help.addString(objectName);
//                help.addString("obs");
                
                // tell LeoGrasper to opn
                
                break;

//		case VOCAB_SET:
//			success = (query.size() > 1) && query.get(1).isString() && hasConfigFile;
//			if (success) {
//				group = file.findGroup(query.get(1).asString());
//				success = controller.bottle2Trajectory(group, graspTrajectory);
//			}
//			if (success) {
//				cout << "Set trajectory to " << query.get(1).asString() << " in configuration file " << endl;
//				response.addString("OK");
//			} else {
//				cout << "Set trajectory failed." << endl;
//				response.addString("FAIL");
//		    }
//			break;
		case VOCAB_QUIT:
            stopped = true;
			break;
		case VOCAB_HELP:
			response.addVocab(Vocab::encode("many"));
//			response.addString("Grasp Controller: \n");
//			response.addString("pre [duration]: perform pregrasp for DURATION seconds.\n");
//			response.addString("cls [speed] [duration] [maxpid]: close with SPEED (angles per \n    second) for DURATION seconds, with proportion MAXPID (0 - 1)\n    squeezing force.\n");
//			response.addString("opn [speed] [duration] [maxpid]: open with SPEED (angles per \n    second) for DURATION seconds, with proportion MAXPID (0 - 1)\n    opening force.\n");
//			response.addString("traj (p_11, ..., p_19), ..., (p_n1, ..., p_n9): \n    set trajectory points 1-n with position of hand\n    joints 1-9 in degrees.\n");
//			response.addString("load NAME: reload configuration file and set trajectory to section NAME. \n    Configuration file location should be specified as command\n    line argument during module startup.\n");
//			response.addString("get: return current trajectory.\n");
//			response.addString("set NAME: set trajectory to section NAME from previously loaded \n    configuration file. Configuration file location should be specified\n    as command line argument during module startup.\n");
			break;
		default:
			response.addString("Unknown Command. Type help for more information.");
			break;
		}

		port.reply(response);
	}
    
    // cleanup
    // TODO
	return 0;
}

bool controlHand(bool open) {
	yarp::os::RpcClient leoGrasperPort;

    // the name of the RPC port used by the core module
    std::string leoGrasperPortName = "/" + robot + "/" + part + "/" + "grasp";
    std::cout << "Trying to connect to LEOGraspPort at: " << leoGrasperPortName << std::endl;
    

    // trying to open our local port
    if (! leoGrasperPort.open("/GraspDemo/leoGraspConnection")) {
        std::cout << "GraspDemo" << ": Unable to open port " << ("/GraspDemo/leoGraspConnection") << std::endl;
        return false;
    }
    
    // trying to connect to the LeoGrasper port
    printf("Trying to connect to %s\n", leoGrasperPortName.c_str());
    if(! Network::connect("/GraspDemo/leoGraspConnection", leoGrasperPortName.c_str()) ) {
        std::cout << "GraspDemo" << ": Unable to connect to port ";
        std::cout << leoGrasperPortName.c_str() << std::endl;
        return false;
    }
    
    yarp::os::Bottle cmd, response;
	
    if(open) cmd.addString("opn");
    else     cmd.addString("cls");
    
	leoGrasperPort.write(cmd, response);
	if( response.get(0).isString() ) {
        if(response.get(0).asString() == "OK") {
            return true;
        }else
            return false;
	}
    
    
    return false;
}
