/*******************************************************************
 ***  Copyright (C) 2010-2015 Mikhail Frank, Juxi Leitner        ***
 ***  CopyPolicy: Released under the terms of the GNU GPL v2.0.  ***
 ******************************************************************/

/** \addtogroup MoBeE
 *	@{
 */

#ifndef PARTCONTROLLER_H
#define PARTCONTROLLER_H

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

#define VOCAB_QATTR VOCAB4('q','a','t','t')
#define VOCAB_QFORCE VOCAB4('q','f','o','r')

/*! \brief Wraps YARP's remote device driver interface neatly
 *  This could be a motor control interface for one part of the iCub for example. see: http://eris.liralab.it/wiki/Motor_control
 */

class PartController : public yarp::os::RateThread
{
	
public:
	
    PartController( const char* _robotName, const char* _partName, const char* _fileName, int r );
    virtual ~PartController();

	bool    isValid();								//!< Checks if the remote device is ready
	int     getNumJoints() { return numJoints; }	//!< Returns the number of controllable axes
    bool    getEncoders( double* q ) { return enc->getEncoders(q); }
    bool    isStopped(double thresh=0.0);
    
private:
	
	yarp::os::Network							network;
	yarp::os::BufferedPort<yarp::os::Bottle>	commandPort;
    yarp::os::BufferedPort<yarp::os::Bottle>    statePort;
	yarp::dev::PolyDriver						*dd;
	yarp::dev::IVelocityControl					*vel;
	yarp::dev::IEncoders						*enc;
    
    bool                controllerIsOn;
    
	int					numJoints;	// number of controllable DOFs
    double              v_gain,t0,t1;
    
	double				*q1,		// current pose
                        *q0,		// last pose
                        *v,			// velocity q1-q0
                        *e,			// the error vector x-q1
    
                        *x,			// the attractor pose
                        *min,		// min joint positions
                        *max,		// max joint positions
    
                        
                        
						*w,			// weight vector indicates importance of DOFs
						*k,			// spring constants for each DOF
						*c,			// damping constants for each DOF
                        *a,			// the acceleration currently felt by the robot
                        *ctrl,		// the next velocity control command
                        
                        *fAtt,         // attractor spring force
                        *fAttMax,      // maximum attractor spring force
                        
                        *fLim,         // joint limit repulsion
                        *limLen,      // length (in degrees) of limit avoidance spring
                        *fLimMax,      // maximum repulsive force
    
                        *fCst,      // linear constraint forces (for use by derrived classes)
                        cstLen,     // length of spring for linear constraint forces
                        *fCstMax,
    
                        *fFld,      // force fields (for use by derrived classes)
                        *kfFld,
                        *fFldMax,
                        
                        
                        
                        *fRPC,         // user defined force (from RPC)
                        *kfRPC,
                        *fRPCMax;
    
protected:
    
    double getCstLen() { return cstLen; }
    
    void setATT(yarp::os::Bottle*);
    void setFFLD(yarp::os::Bottle*);
    void setFCST(yarp::os::Bottle*);
    void setFRPC(yarp::os::Bottle*);
    
    virtual void handler( yarp::os::Bottle* ){}
    virtual void procEncoders( double* ){}
    virtual void publishState(){}
    virtual bool threadInit();
	virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
    
    double magnitude(yarp::os::Bottle*);
	
    yarp::os::ConstString portPrefix;
};
#endif
/** @} */
