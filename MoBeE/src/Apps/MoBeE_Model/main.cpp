/*******************************************************************
 ***  Copyright (C) 2011-2015 Mikhail Frank, Juxi Leitner        ***
 ***  CopyPolicy: Released under the terms of the GNU GPL v2.0.  ***
 ******************************************************************/

#include <QApplication>
#include <yarp/os/Property.h>

//#include "yarpRobot.h"
//#include "yarpModel.h"
#include "controller.h"
// #include "simSyncer.h"

#include <unistd.h>
#include "robot.h"
#include "model.h"


using namespace KinematicModel;

int main(int argc, char *argv[])
{
    /******************************************************************************
     ******************************************************************************/
    
    // whether or not to run the visualization
    // TODO for development
    // --robot ~/Code/iCub/MoBeE/xml/icubSim.xml
    // --visualize
    bool visualize = true;
    
    // path to the XML file that defines the robot model
    QString robotFile = "/Users/leitnerj/Code/iCub/MoBeE/xml/Baxter.xml";
    //QString robotFile2 = config.find("robot2").asString().c_str();
    QString confDir = "/Users/leitnerj/Code/iCub/MoBeE/simConf/";
    
    QApplication app( argc, argv, visualize );	// create the QT application
    
    Model model( visualize );
    model.start();	/* if we want display lists to be created automatically,
                     the model must be started prior to appending objects */
    
    // do we need to wait here or something?!
    
    
    //    robotFile = argv[1];
    printf( "loading robot file: %s\n", robotFile.toStdString().c_str() );
    //MoBeE::Yarp
    Robot *robot = model.loadYarpRobot( robotFile, false );
    // don't ask me why I need this!
    robot->setName(robot->getName().c_str());
    printf( "robot <%s> is loaded\n", robot->getName().c_str());

    
    QVector<Controller*> controllers;
    if( robot ) {
        
        printf("done loading robot\n");
        printf( "\nLOADING CONTROLLER CONFIG FILES FROM: %s\n", confDir.toStdString().c_str() );
        
        for ( int i=0; i < robot->numBodyParts(); i++ )
        {
            QString config_file;
            if (!confDir.isEmpty()) config_file = confDir + "/" + robot->getPart(i)->name() + ".ini";
            Controller* c = new Controller( robot, config_file, i, 20 );
            controllers.append(c);
            c->start();
        }
    }
    #ifdef WIN32
        Sleep(1);
    #else
        usleep(1000);
    #endif
    //printf( "loading world file: %s\n", argv[1] );
    //model.loadWorld( QString(argv[2]), false );
    
    // ObjectSoup soup( model, 5, false );
    // soup.start();
    
    
    int result = app.exec();						// run the Qt application
    
    // soup.stop();
    model.stop();
    
    return result;
}


//
//int main(int argc, char *argv[])
//{
//	/******************************************************************************
//	*** Create a configuration for the robot model from command line arguments	***
//	***	currently supported arguments are:										***
//	***		--file yourRobotModel.xml (required)								***
//	***		--visualize															***
//	******************************************************************************/
//	
//    yarp::os::Property config;
//    config.fromCommand(argc,argv);
//    
//    // path to the XML file that defines the robot model
//    QString robotFile = config.find("robot").asString().c_str();
//    //QString robotFile2 = config.find("robot2").asString().c_str();
//
//    // path to config dir for robot controllers
//    QString confDir = config.find("conf").asString().c_str();
//
//    // optional... path to the xml file that contains the world
//    QString worldFile = config.find("world").asString().c_str();
//    
//    // whether or not to run the visualization
//    bool visualize = false;
//    if ( config.check("visualize") ) visualize = true;
//
//    // print the configuration to the console
//    printf("Launching MoBeE... \n");
//    if ( visualize ) {	printf("  ...with visualization\n");		}
//    else {				printf("  ...without visualization\n");		}
//	
//	/***********************************************************************/
//	
//	// Create the QApplication
//	QApplication app( argc, argv, visualize );	// create the QT application
//	
//	// MoBeE::YarpModel* yarpModel = NULL;
//	// MoBeE::YarpRobot* yarpRobot = NULL;
//    // QVector<Controller*> controllers;
//
//	// int result = 0;
//	
//	// try
//	// {
// //        printf( "loading robot file: %s\n", argv[1] );
//        
// //        yarpModel = new MoBeE::YarpModel ( visualize );
// //        yarpModel->start();
// //                            /*	if we want display lists to be created automatically,
// //                            the model must be started prior to appending objects by
// //                            calling loadWorld(), loadRobot(), or appendObject()		*/
//
// //        // Load a robot model from file
// //        if ( robotFile != "" )
// //        {
// //            printf( "\nLOADING ROBOT MODEL FROM: %s\n", robotFile.toStdString().c_str() );
//            
// //            yarpRobot = yarpModel->loadYarpRobot( robotFile, false );
// //            printf("name: %s\n", yarpRobot->getName().c_str());
//        
// //            if( yarpRobot ) {
// //                printf("done loading robot\n");
//                
// //                printf( "\nLOADING CONTROLLER CONFIG FILES FROM: %s\n", confDir.toStdString().c_str() );
// //                for ( int i=0; i < yarpRobot->numBodyParts(); i++ )
// //                {
// //                    QString config_file;
// //                    if (!confDir.isEmpty()) config_file = confDir + "/" + yarpRobot->getPart(i)->name() + ".ini";
// //                    Controller* c = new Controller( yarpRobot, config_file, i, 20 );
// //                    controllers.append(c);
// //                    c->start();
// //                }
// //            }
// //          /*#ifdef WIN32
// //            Sleep(1);
// //          #else
// //            usleep(1000);
// //          #endif*/
// //        }
//
// //        // Load a world config from file
// //        if ( worldFile != "" )
// //        {
// //            printf( "\nLOADING WORLD MODEL FROM: %s\n", worldFile.toStdString().c_str() );
// //            yarpModel->loadWorld( worldFile, false );
// //        }
// //        else    printf( "\nno world model loaded\n");
//	
//	// 	// run the Qt application
//	// 	result = app.exec();
//
// //        for ( size_t i=0; i < controllers.size(); i++ )
// //            controllers.at(i)->stop();
//        
//	// 	if ( yarpModel ) {
// //            yarpModel->closeWorldRpcPort();
//	// 		yarpModel->stop();
//	// 		delete yarpModel;
//	// 	}
//	// }
//	// catch ( std::exception& e )
//	// { 
//	// 	printf("%s\n", e.what());
//	// 	return 1;
//	// }
//
//
//
//// 	yarpModel = new MoBeE::YarpModel( visualize );
//// 	yarpModel->start();	/* if we want display lists to be created automatically,
//// 					   the model must be started prior to appending objects */
//	
//// 	printf( "loading robot file: %s\n", argv[1] );
//// 	yarpRobot = yarpModel->loadYarpRobot( robotFile, false );
//
//// 	//printf( "loading world file: %s\n", argv[1] );
//// 	//model.loadWorld( QString(argv[2]), false );
//	
//// 	// ObjectSoup soup( model, 5, false );
//// 	// soup.start();
//
//// 	int result = app.exec();						// run the Qt application
//	
//// 	// soup.stop();
//// 	yarpModel->stop();
//
//// 	delete yarpModel;
//	
////     printf("All finished\n");
////     return result;
//// }
//
//
//	
//    
//    // whether or not to run the visualization
//    // TODO for development
//    // --robot ~/Code/iCub/MoBeE/xml/icubSim.xml
//    // --visualize
//    
//    // path to the XML file that defines the robot model
//    //QString robotFile = "/Users/leitnerj/Code/iCub/MoBeE/xml/icubSim.xml";
//    //QString robotFile2 = config.find("robot2").asString().c_str();
//    
////    Model model( visualize, false );
//    MoBeE::YarpModel model( visualize );
//
//    model.start();	/* if we want display lists to be created automatically,
//                     the model must be started prior to appending objects */
//    
//    //    robotFile = argv[1];
//    printf( "loading robot file: %s\n", robotFile.toStdString().c_str() );
//    model.loadRobot( robotFile, false );
//    
//    //printf( "loading world file: %s\n", argv[1] );
//    //model.loadWorld( QString(argv[2]), false );
//    
//    // ObjectSoup soup( model, 5, false );
//    // soup.start();
//    
//    
//    int result = app.exec();						// run the Qt application
//    
//    // soup.stop();
//    model.stop();
//    
//    return result;
//
//}
