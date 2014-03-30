// Copyright: (C) 2014 Juxi Leitner
// Author: Juxi Leitner <juxi.leitner@gmail.com>
// find more information at http://Juxi.net/
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include "yarp_helpers.h"

using namespace yarp::os;
using namespace yarp::dev;


yarp::os::BufferedPort<yarp::os::Bottle> right_hand, left_hand;
yarp::os::BufferedPort<yarp::os::Bottle> left_grasp_port, right_grasp_port;

bool yarp_init(yarp::os::Property &cmd) {
    std::cout << "Setting up YARP... ";

    Network yarp;    
	
    // check for configuration file
    Property file;

	// the following values are read from the configuration file, but can be overwritten from the command line
	std::string robot = "MoBeE"; // default is simulator

	// open left end effector port
	std::string portname_r, portname_l, part;	// whether or not to use the left hand (default is right)

	// a port for sending commands to MoBeE right arm
	part = "right_arm";
    portname_l = "/SDLCtrlr/" + part + "/" + "cmd:o";	    
    portname_r = "/" + robot + "/" + part + "/" + "cmd:i";	// the name of the RPC server	
    right_hand.open(portname_l.c_str());
    yarp::os::Network::connect(portname_l.c_str(), portname_r.c_str());    

	// a port for sending commands to MoBeE left arm
    part = "left_arm";
    portname_l = "/SDLCtrlr/" + part + "/" + "cmd:o";	    
    portname_r = "/" + robot + "/" + part + "/" + "cmd:i";	// the name of the RPC server	
    left_hand.open(portname_l.c_str());
    yarp::os::Network::connect(portname_l.c_str(), portname_r.c_str());    
    
    // setup the grasp ports
    robot = "icub";
    // the name of the RPC port used by the core module
    portname_r = "/" + robot + "/" + part + "/" + "grasp";
    portname_l = "/SDLCtrlr/" + part + "/" + "grasp:o";	    
    left_grasp_port.open(portname_l.c_str());
    yarp::os::Network::connect(portname_l.c_str(), portname_r.c_str());    

	part = "right_arm";
    // the name of the RPC port used by the core module
    portname_r = "/" + robot + "/" + part + "/" + "grasp";
    portname_l = "/SDLCtrlr/" + part + "/" + "grasp:o";	    
    right_grasp_port.open(portname_l.c_str());
    yarp::os::Network::connect(portname_l.c_str(), portname_r.c_str());        
    
    // Finished reading configuration file
    std::cout << "\tdone!" << std::endl << std::endl;
    
    return true;
}

void yarp_cleanup() {
    right_hand.interrupt();
    left_hand.interrupt();
    right_grasp_port.interrupt();
    left_grasp_port.interrupt();

    right_hand.close();
    left_hand.close();
    right_grasp_port.close();
    left_grasp_port.close();


    // Finished reading configuration file
    std::cout << "YARP clean-up... \tdone!" << std::endl;
}


// left/right, up/down
bool yarp_send_force(int arm_offset, int lr, int ud, int fb) {    
    // arm_offset == 0 -> left 
    // arm_offset == 2 -> right
    
    yarp::os::BufferedPort<yarp::os::Bottle> *part = NULL;
    
    if(arm_offset == 0) part = &left_hand;
    if(arm_offset == 2) part = &right_hand;

    yarp::os::Bottle& msg = part->prepare();
    // get a bottle from YARP, in which to put our control command
    msg.clear();
    // indicates an operational space control command
    msg.addVocab(yarp::os::Vocab::encode("opsp"));

    // to be applied to the 'marker' called 'right_hand' or 'left_hand'
    if(arm_offset == 0) msg.addString("left_hand");
    if(arm_offset == 2) msg.addString("right_hand");
    
    if( lr < -600 || lr > 600 ) {
        printf("value out of range, lr: %d\n", lr);
        return false;
    }
    
    if( ud < -600 || ud > 600 ) {
        printf("value out of range, ud: %d\n", ud);
        return false;
    }

    // a bottle to represent a force vector
    yarp::os::Bottle cmd;
    cmd.addDouble( fb * 1.25 ); //fb * 1.50 ); // x -> forward!
    cmd.addDouble( lr * 2.85 );
    cmd.addDouble( ud * 2.85 );

    // nest the force vector inside our control command bottle
    msg.addList() = cmd;

    // send the command to MoBeE
    part->write();

    // inform the user
    // printf("Sending OpSp Force Command: %s\n", msg.toString().c_str());
    
    return true;
}


bool yarp_send_grasp(int arm_offset, bool open) {
    printf("sending grasp %d %d\n", arm_offset, open);
    yarp::os::BufferedPort<yarp::os::Bottle> *port;
    if(arm_offset == 0) port = &left_grasp_port;
    if(arm_offset == 2) port = &right_grasp_port;    

    yarp::os::Bottle& msg = port->prepare();

    if(open) msg.addString("opn");
    else     msg.addString("cls");
    
    port->write();
//    printf("%s\n", toString().c_str());    
    //, response);
    // if( response.get(0).isString() ) {
    //     if(response.get(0).asString() == "OK")
    //         return true;
    // }

    return false;
}

#define VOCAB_START VOCAB4('s','t','a','r')
#define VOCAB_FORCE VOCAB4('f','o','r','c')

#define VOCAB_HELP VOCAB4('h','e','l','p')	// display help
#define VOCAB_QUIT VOCAB4('q','u','i','t')


