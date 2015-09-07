/*******************************************************************
 ***  Copyright (C) 2011-2015 Mikhail Frank, Juxi Leitner        ***
 ***  CopyPolicy: Released under the terms of the GNU GPL v2.0.  ***
 ******************************************************************/

#include <QApplication>
#include "model.h"
#include <yarp/os/Property.h>

using namespace KinematicModel;

int main(int argc, char *argv[])
{
    /******************************************************************************
     *** Create a configuration for the robot model from command line arguments	***
     ***	currently supported arguments are:										***
     ***		--file yourRobotModel.xml (required)								***
     ***		--visualize															***
     ******************************************************************************/
    
    yarp::os::Property config;
    config.fromCommand(argc,argv);
    
    // path to the XML file that defines the robot model
    QString robotFile = config.find("robot").asString().c_str();
    //QString robotFile2 = config.find("robot2").asString().c_str();
    
    // path to config dir for robot controllers
    QString confDir = config.find("conf").asString().c_str();
    
    // optional... path to the xml file that contains the world
    QString worldFile = config.find("world").asString().c_str();
    
    // whether or not to run the visualization
    bool visualize = false;
    if ( config.check("visualize") ) visualize = true;
    
    // print the configuration to the console
    printf("Launching MoBeE... \n");
    if ( visualize ) {	printf("  ...with visualization\n");		}
    else {				printf("  ...without visualization\n");		}
    
    /***********************************************************************/

	
	QApplication app( argc, argv, visualize );	// create the QT application
	
	Model model( visualize, false );
	model.start();	/* if we want display lists to be created automatically,
					   the model must be started prior to appending objects */
	
	printf( "loading robot file: %s\n", argv[1] );
    Robot *robot =    model.loadRobot( robotFile, false );

    //robot->setName("test");
    robot->sayMyName();
    std::string name = robot->getName();
    printf("name: %s", name.c_str());


	//printf( "loading world file: %s\n", argv[1] );
	//model.loadWorld( QString(argv[2]), false );
	
	// ObjectSoup soup( model, 5, false );
	// soup.start();

	int result = app.exec();						// run the Qt application
	
	// soup.stop();
	model.stop();
    	
	return result;
}
