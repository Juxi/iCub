/*******************************************************************
 ***               Copyright (C) 2011 Mikhail Frank              ***
 ***  CopyPolicy: Released under the terms of the GNU GPL v2.0.  ***
 ******************************************************************/

#include "modelexception.h"

using namespace KinematicModel;

KinematicModelException::KinematicModelException( const QString& msg )
{
	errStr = "KinematicModelException: ";
	errStr.append(msg);
}
KinematicModelException::~KinematicModelException() throw()
{
}

const char* KinematicModelException::what() const throw()
{
	return errStr.toStdString().c_str();
}