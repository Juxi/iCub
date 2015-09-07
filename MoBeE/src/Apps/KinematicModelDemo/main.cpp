/*******************************************************************
 ***  Copyright (C) 2011-2015 Mikhail Frank, Juxi Leitner        ***
 ***  CopyPolicy: Released under the terms of the GNU GPL v2.0.  ***
 ******************************************************************/

#include <QApplication>
#include "model.h"
#include "box.h"
#include "sphere.h"
#include "cylinder.h"
#include "box.h"
#include "objectsoup.h"

using namespace KinematicModel;

int main(int argc, char *argv[])
{
	bool visualize = true;
	
	QApplication app( argc, argv, visualize );	// create the QT application
	
	Model model( visualize, false );
	model.start();	/* if we want display lists to be created automatically,
					   the model must be started prior to appending objects */
	
	printf( "loading robot file: %s\n", argv[1] );
    Robot *robot =    model.loadRobot( QString(argv[1]), false );

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
