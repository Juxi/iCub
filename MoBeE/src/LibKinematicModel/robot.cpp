/*******************************************************************
 ***  Copyright (C) 2011-2015 Mikhail Frank, Juxi Leitner        ***
 ***  CopyPolicy: Released under the terms of the GNU GPL v2.0.  ***
 ******************************************************************/

#include "robot.h"
#include "model.h"
#include "box.h"
#include <time.h>
#include <iostream>


using namespace KinematicModel;

int recursion_count;


Robot::Robot( Model* m, DT_RespTableHandle robotTable,
                        DT_RespTableHandle fieldTable,
                        DT_ResponseClass robotClass,
                        DT_ResponseClass baseClass,
                        DT_ResponseClass robotField,
                        DT_ResponseClass robotBaseField//,                        bool _openWithField
                                        ) :
                                            robotName("unNamedRobot"),
                                            model(m),
                                            responseTable(robotTable),
                                            fieldResponseTable(fieldTable),
                                            worldRobotClass(robotClass),
                                            worldBaseClass(baseClass),
                                            worldFieldClass(robotField),
                                            worldBaseFieldClass(robotBaseField),
                                            numCompositObjects(0),
                                            isConfigured(false),
                                            //openWithField(_openWithField),
                                            numCollisions(0),
                                            numReflexCollisions(0),
                                            myName ("myName")
{
	if ( !robotTable ) { throw KinematicModelException("The Robot constructor requires a valid DT_RespTableHandle."); }
	qRegisterMetaType< QVector<qreal> >("QVector<qreal>");
	qRegisterMetaType< RobotObservation >("RobotObservation");    
}
Robot::~Robot()
{
	if ( isOpen() ) { close(); }
    DT_DestroyRespTable(getResponseTable());
    DT_DestroyRespTable(getFieldResponseTable());
}

void Robot::appendNode( KinTreeNode* node )
{
	//printf("Called Append KinTreeNode to Robot\n");
    printf("size: %d\n", tree.size());
	tree.append(node);
}

void Robot::close()
{	
	isConfigured = false;
	
	QVector<Motor*>::iterator j;
    for ( j=motorList.end(); j!=motorList.begin(); ) {
		--j;
        delete (*j);
    }
    QVector<BodyPart*>::iterator i;
    for ( i=partList.end(); i!=partList.begin(); )
	{
		--i;
        delete (*i);
    }
	
	kill();
}

void Robot::kill()
{
	QVector<KinTreeNode*>::iterator i;
    for ( i=tree.begin(); i!=tree.end(); ++i )
	{
        (*i)->kill();
    }
}

void Robot::open(const QString& fileName, bool verbose) throw(KinematicModelException)
{
    printf("Robot.Open()\n");
    
    QFile file(fileName);
    if ( !file.open(QFile::ReadOnly | QFile::Text) )
    {
        QString errStr = "failed to open robot file '";
        errStr.append(fileName);
        errStr.append("'");
        throw KinematicModelException(errStr);
    } //else printf("text file found\n");
    
    if( fileName.endsWith(".xml") ) {
        openXML(file, verbose);
    } else if( fileName.endsWith(".urdf") ) {
        openURDF(file, verbose);
    } else {
        printf("Error! The file extension is unknown, try using .xml for ZeroPosition"
               "descriptions or .urdf for Unified Robot Description Format (URDF) descriptions.\n");
    }
    printf("Parsed the robot file\n");
    
    ignoreAdjacentPairs();
    home();
    
    printf("Created Robot: %s (%d kinematic tree nodes, %d primitives)\n", getName().c_str(), numCompositObjects, getNumPrimitives());
    
    printf("robot created");
    isConfigured = true;
}

void Robot::openXML(QFile& file, bool verbose) throw(KinematicModelException)
{
    //printf("Robot.OpenXML()\n");
    ZPHandler handler( model, this );
    QXmlSimpleReader reader;
    reader.setContentHandler(&handler);
    reader.setErrorHandler(&handler);
    
    QXmlInputSource xmlInputSource( &file );
    if ( !reader.parse( xmlInputSource ) )
    {
        QString errStr = "failed to create robot '";
//        errStr.append(getName());
        errStr.append("' from file '");
        errStr.append(file.fileName());
        errStr.append("'");
        throw KinematicModelException(errStr);
    } //else printf("text file parsed\n");
    
    //printf("Parsed the robot file\n");
    //
    //	ignoreAdjacentPairs();
    //	home();
    //
    //	printf("Created Robot: %s (%d kinematic tree nodes, %d primitives)\n",getName().toStdString().c_str(), numCompositObjects, getNumPrimitives());
    //
    //	isConfigured = true;
}

void Robot::openURDF(QFile& file, bool verbose) throw(KinematicModelException)
{
    printf("Robot.OpenURDF()\n");
    
#ifdef ENABLE_URDF
    urdf::Model urdfModel;
    if (! urdfModel.initFile(file.fileName() ) ) {
        throw new KinematicModelException();
    }
#else
#ifdef ENABLE_URDFDOM
    
    // Load the URDF file
    boost::shared_ptr<urdf::ModelInterface> urdf = urdf::parseURDFFile(file.fileName().toStdString());
    
    ZPHandler *hdl = new ZPHandler(model, this);
    
    // set the robot name
    setName(QString::fromStdString(urdf->name_));
    
    /** We need a body part, maybe this needs to be investigated a bit closer **/
    // hack: look at this
    BodyPart* existingBodyPart;
    if ( (existingBodyPart = getPartByName(QString(getName().c_str()))) != NULL ) {
        hdl->bodyPart = existingBodyPart; // if the branch by this name already exists, use the prexisting one
    } else {
        // make a new one
        if ( !(hdl->bodyPart = hdl->createChildPart()) ) {
            printf("ERROR! Could not create a body part!");
        }
        hdl->bodyPart->setName( getName().c_str() );
    }
    
    if ( !hdl->bodyPart->verify() ) {
        printf("ERR! Body part contains null pointers. Check index assignments. When assigning manually, every index 0..n must be used.");
    }
    
    // ROOT Node
    printf("Root: %s [child: jnts: %lu, lnks: %lu]\n",
           urdf->getRoot()->name.c_str(),
           urdf->getRoot()->child_joints.size(),
           urdf->getRoot()->child_links.size());
    
    // THINK: do we need a bodypart?!?
    // FIXIT: we can only have one bodyparthere!
    BodyPart *bodyPart = NULL;
    recursion_count = 0;
    if ( (bodyPart = hdl->createChildPart()) ) {
        bodyPart->setName( urdf->getRoot()->name.c_str() );
        URDFparseLink(hdl, urdf, urdf->getRoot()->name);
        bodyPart = bodyPart->parent();
    }
    
#else
    printf("URDF parser is not enabled! (Libraries are missing!)");
#endif
#endif
}

void Robot::URDFparseLink(ZPHandler *hdl, boost::shared_ptr<urdf::ModelInterface> urdf, std::string linkName) {
    boost::shared_ptr< const urdf::Link > xml = urdf->getLink(linkName);

    recursion_count++;
    for(int i = 0; i < recursion_count; i++) printf("\t");
    printf("-----------------------------------------------\n");
    for(int i = 0; i < recursion_count; i++) printf("\t");
    printf("Link: %s [child: jnts: %lu, lnks: %lu]\n",
           xml->name.c_str(),
           xml->child_joints.size(),
           xml->child_links.size());
    
    if( xml->getParent() == NULL) {
        // orgin? root node?
        origin.clear();
    }
    
    for(int i = 0; i < recursion_count; i++) printf("\t");
    printf("Origin: %f, %f, %f\n", origin.position.x, origin.position.y, origin.position.z);

    //////////////////////////////////////////////
    
        // THIS IS TODO!!
    
    //////////////////////////////////////////////
    
    // does the link have a visual element assigned?
    if( xml->visual != NULL && xml->visual->geometry != NULL) {
        
        // look at code from here on down to do fancy maths ;)
        // and make it work!
        
        
        PrimitiveObject* primitive = URDFparseGeometry(xml->visual->geometry);
        if( primitive != NULL ) {
            for(int i = 0; i < recursion_count; i++) printf("\t");
            printf("=========== new primitive ----------- \n");
            // only if we have a valid primitive it is added to the node

            hdl->node = hdl->createChildLink();
            //          hdl->node->setNodeAxis(mainAxis);
            for(int i = 0; i < recursion_count; i++) printf("\t");
            printf("NodeAxis: %f, %f, %f", hdl->node->nodeAxis.x(), hdl->node->nodeAxis.y(), hdl->node->nodeAxis.z());
            // TODO: FIX: does the element have also an origin tag?
            
            
            urdf::Pose pose;
            // origin (from Joint) is added to current readouts to define the pose
            
            //            /*
            //             *	SET ITS ROTATION
            //             */
            QVector3D rollAxis  (1, 0, 0);
            QVector3D pitchAxis (0, 1, 0);
            QVector3D yawAxis   (0, 0, 1);
            
            QVector3D heightAxis;           //xml->visual->origin.position.x,
            //xml->visual->origin.position.y,
            //xml->visual->origin.position.z );
            //            if ( qFuzzyIsNull( heightAxis.length() ) ) { heightAxis.setY(1); }
            heightAxis.setZ(1);
            //
            double roll, pitch, yaw;
            xml->visual->origin.rotation.getRPY(roll, pitch, yaw);  //in RAD;
            
            for(int i = 0; i < recursion_count; i++) printf("\t");
            printf("rpy: %f, %f, %f\n", roll, pitch, yaw);
            
            qreal rotAngle = 0.0;
            if( !qFuzzyIsNull(roll) ) {  rotAngle = roll;  heightAxis = rollAxis; }
            if( !qFuzzyIsNull(pitch) ){  rotAngle = pitch; heightAxis = pitchAxis; }
            if( !qFuzzyIsNull(yaw) )  {  rotAngle = yaw;   heightAxis = yawAxis; }
            
            //primitive->setCartesianOrientation(QVector3D(0,0,0));
            //
            for(int i = 0; i < recursion_count; i++) printf("\t");
            printf("Orientation: %f, %f\n", heightAxis.length(), rotAngle);
            //primitive->setSpecialEulerOrientation(heightAxis, rotAngle);
            primitive->setSpecialEulerOrientation( heightAxis, rotAngle );
            
            /*
             *	SET ITS POSITION
             */
            //            QVector3D position = QVector3D( xml->visual->origin.position.x,
            //                                           xml->visual->origin.position.y,
            //                                           xml->visual->origin.position.z );
            pose.position = origin.position + xml->visual->origin.position;
            QVector3D position = QVector3D( pose.position.x,
                                            pose.position.y,
                                            pose.position.z );

//            primitive->translate(position);
//            printf("primitive translation %f, %f, %f\n", position.x(), position.y(), position.z());
            //primitive->setOpaque();
        
            
            
            // printf("\tOrigin x: %f\n", xml->visual->origin.position.x);
            //            QVector3D axis = QVector3D( xml->visual->origin.position.x,
            //                                       xml->visual->origin.position.y,
            //                                       xml->visual->origin.position.z );
            // for joints only? hdl->node->setNodeAxis(axis);
            
            //    height = attributes.value("length").toDouble();
            // if(xml->collision != NULL) rField = 1.0;
            //        if ( qFuzzyIsNull(height) ) { height = axis.length(); }
            //
            //    axis = height * axis.normalized();
            
            
            
            //            if(primitive != NULL) {
            hdl->node->appendPrimitive(primitive);

            for(int i = 0; i < recursion_count; i++) printf("\t");
            printf("Adding primitive... \n");
            //            }
            
        } else {
            printf("No visual tag or geometry tag found for node \"%s\" in the URDF file!", xml->name.c_str());
        }
        
        if( xml->child_joints.size() > 0 ) {
            // Create properties for each joint.
            typedef std::vector<boost::shared_ptr<urdf::Joint> > M_NameToUrdfJoint;
            M_NameToUrdfJoint::const_iterator joint_it = xml->child_joints.begin();
            M_NameToUrdfJoint::const_iterator joint_end = xml->child_joints.end();
            for( ; joint_it != joint_end; ++joint_it )
            {
                const boost::shared_ptr<const urdf::Joint>& urdf_joint = *joint_it;

                for(int i = 0; i < recursion_count; i++) printf("\t");
                printf("JointName: %s -> %s\n",
                       urdf_joint->name.c_str(),
                       urdf_joint->child_link_name.c_str());
                
                urdf::Pose old_origin = origin;
//                origin.position.x += urdf_joint->parent_to_joint_origin_transform.position.x;
//                origin.position.y += urdf_joint->parent_to_joint_origin_transform.position.y;
//                origin.position.z += urdf_joint->parent_to_joint_origin_transform.position.z;
                origin = urdf_joint->parent_to_joint_origin_transform;
//                origin = origin.position + urdf_joint->parent_to_joint_origin_transform;
                for(int i = 0; i < recursion_count; i++) printf("\t");
                printf("Jnt moved origin to: %f, %f, %f\n", origin.position.x, origin.position.y, origin.position.y);
                //            joints_[urdf_joint->name] = joint;
                //
                //            joint->setRobotAlpha( alpha_ );
                primitive->translate(QVector3D(origin.position.x, origin.position.y, origin.position.z));
                
                // recurse into it!
                URDFparseLink(hdl, urdf, urdf_joint->child_link_name);
                
                // resetting origin
                for(int i = 0; i < recursion_count; i++) printf("\t");
                printf("resetting origin");
                origin = old_origin;
            }
        }
        
    }
 
    
    if( hdl->node != NULL) {
        model->appendObject(hdl->node);
        countCompositeObject();
        hdl->node = hdl->node->parent();
    }
    
    for(int i = 0; i < recursion_count; i++) printf("\t");
    printf("EndOrigin: %f, %f, %f\n", origin.position.x, origin.position.y, origin.position.z);
    

    for(int i = 0; i < recursion_count; i++) printf("\t");
    printf("Link FINISHED: %s\n",
           xml->name.c_str());
    for(int i = 0; i < recursion_count; i++) printf("\t");
    printf("Origin: %f, %f, %f\n", origin.position.x, origin.position.y, origin.position.z);
    
    recursion_count--;
}

void Robot::URDFparseJoint(ZPHandler *hdl, boost::shared_ptr<urdf::ModelInterface> urdf, std::string jointName) {
    boost::shared_ptr< const urdf::Joint > xml = urdf->getJoint(jointName);
    printf("Joint: %s \n", xml->name.c_str());
    printf("Joint finished!");
}

PrimitiveObject* Robot::URDFparseGeometry (boost::shared_ptr<urdf::Geometry> geom) {
//    printf("\tLink Geometry Type: %d\t", geom->type);
    
    PrimitiveObject* primitive;
    try
    {
        if ( geom->type == urdf::Geometry::SPHERE ) {
            boost::shared_ptr<urdf::Sphere> sph = boost::dynamic_pointer_cast<urdf::Sphere>(geom);
            primitive = new KinematicModel::Sphere( sph->radius );
        }
        else if ( geom->type == urdf::Geometry::CYLINDER ) {
            boost::shared_ptr<urdf::Cylinder> cyl = boost::dynamic_pointer_cast<urdf::Cylinder>(geom);
            primitive = new KinematicModel::Cylinder( cyl->radius, cyl->length );
            QVector3D upright = QVector3D( M_PI_2, 0.0, 0.0 );
            primitive->setCartesianOrientation( upright );
            //primitive->setAxisAngleOrientation( position, 0.0 );
        }
        else if ( geom->type == urdf::Geometry::BOX ) {
            boost::shared_ptr<urdf::Box> box = boost::dynamic_pointer_cast<urdf::Box>(geom);
            QVector3D size = QVector3D(box->dim.x,
                                       box->dim.y,
                                       box->dim.z);
            primitive = new KinematicModel::Box( size );
            printf("\tBox %d\n", primitive->getGeomType());
//            QVector3D upright = QVector3D( M_PI_2, 0.0, 0.0 );
//            primitive->setCartesianOrientation( upright );
        }
        else if (geom->type == urdf::Geometry::MESH) {
            printf("MESH!!!\n");
            // FIX: TODO: THINK about loading the mesh
            
            primitive = NULL;
        }
        else { printf("unknown primitive geometry!\n");
            std::exception e;
            throw  e; }
    }
    catch (std::exception& e)
    {
        printf("%s\n", e.what());
        return NULL;
    }
    
    return primitive;
}


/*void Robot::removeCollisionResponse( DT_ResponseClass c, DT_RespTableHandle t );
{
	QVector<KinTreeNode*>::iterator i;
    for ( i=tree.begin(); i!=tree.end(); ++i )
	{
        (*i)->removeCollisionResponse( c, t );
    }
}*/

void Robot::ignoreAdjacentPairs()
{
    QVector<KinTreeNode*>::iterator i;
    for ( i=tree.begin(); i!=tree.end(); ++i )
	{
        (*i)->ignoreAdjacentPairs();
    }
}

void Robot::home(bool verbose)
{
	if (verbose) printf("Going to home position.\n");
	QVector<Motor*>::iterator j;
    for ( j=motorList.begin(); j!=motorList.end(); ++j ) {
        (*j)->home(verbose);
    }
	//updatePose();
}

void Robot::resetExtTorque()
{
    QVector<Motor*>::iterator j;
    for ( j=motorList.begin(); j!=motorList.end(); ++j ) {
        (*j)->resetTorque();
    }
}

void Robot::setNormalPosition( qreal pos )
{
	//QMutexLocker locker(&mutex);
	
    QVector<BodyPart*>::iterator i;
    QVector<Motor*>::iterator j;
	
	//printf("Setting all positions to %f\n", pos);
    for ( i=partList.begin(); i!=partList.end(); ++i ) {
        for ( j = (*i)->begin(); j != (*i)->end(); ++j ) {
            (*j)->setNormPos(pos);
        }
    }
}
void Robot::setEncoderPosition( qreal pos )                                        // for every motor on the robot
{
	QMutexLocker locker(&mutex);
	
    QVector<BodyPart*>::iterator i;
    QVector<Motor*>::iterator j;

	//printf("Setting all positions to %f\n", pos);
    for ( i=partList.begin(); i!=partList.end(); ++i ) {
        for ( j = (*i)->begin(); j != (*i)->end(); ++j ) {
            (*j)->setEncPos(pos);
        }
    }
}

bool Robot::setEncoderPosition(int partNum, const QVector<qreal>& pos)           // for an entire branch (using encoder positions)
{
	//printf("called setEncoderPosition() - size %i \n", pos.size() );

	QMutexLocker locker(&mutex);
	
    if ( partIdxInRange(partNum) ) {
        return partList.at(partNum)->setEncPos(pos);
    }
    else { return 0; }
}

bool Robot::setNormalPosition( int partNum, int motorNum, qreal pos )           // for only one motor (using normal position)
{
	QMutexLocker locker(&mutex);
	
    if ( partIdxInRange(partNum) && motorIdxInRange(partNum,motorNum) ) {
        partList.at(partNum)->at(motorNum)->setNormPos(pos);
        return 1;
    }
    else { return 0; }
}

bool Robot::setEncoderPosition( int partNum, int motorNum, qreal pos )           // for only one motor (using encoder position)
{
	QMutexLocker locker(&mutex);
	
    if ( partIdxInRange(partNum) && motorIdxInRange(partNum,motorNum) ) {
        partList.at(partNum)->at(motorNum)->setEncPos(pos);
        return 1;
    }
    else { return 0; }
}

qreal Robot::EncoderToNormalPosition( int partNum, int motorNum, qreal pos ) {
    if ( partIdxInRange(partNum) && motorIdxInRange(partNum,motorNum) ) {
		return partList.at(partNum)->at(motorNum)->encToNorm(pos);
    }
    else
    	return 0.0;
}

qreal Robot::NormalToEncoderPosition( int partNum, int motorNum, qreal pos ) {
	if ( partIdxInRange(partNum) && motorIdxInRange(partNum,motorNum) ) {
		return partList.at(partNum)->at(motorNum)->normToEnc(pos);
	}
	else
		return 0.0;
}

void Robot::updatePose()
{
	//QMutexLocker locker(&mutex);
	
    QMatrix4x4 T;
    T.setToIdentity();
    QVector<KinTreeNode*>::iterator i;
    for ( i=tree.begin(); i!=tree.end(); ++i ) {
        (*i)->update(T);
    }
	
	unsigned int m, mc = markers.size();
	for (m=0; m<mc; m++) {
		markers[m]->update();
	}
	
	//emit changedState();
}


/*void setColliding(KinTreeNode* node) { // recursively set the whole node to colliding mode
	const QVector<PrimitiveObject*>& primitives = node->data();
	QVector<PrimitiveObject*>::const_iterator k;
	for (k = primitives.begin(); k!=primitives.end(); ++k) {
		(*k)->setColliding(CONSTRAINT);
	}
	//const QVector<KinTreeNode*>& children = node->childnodes();
	//QVector<KinTreeNode*>::const_iterator c;
	//for (c = children.begin(); c!=children.end(); ++c) {
	//	setColliding((*c));
	//}
}*/

void Robot::evaluateConstraints()
{
	QVector<BodyPart*>::iterator i;
	QVector<Motor*>:: iterator m;
	QVector<Joint*>:: iterator j;
    QVector<PrimitiveObject*>::const_iterator k;
	

    // VISUALIZE CONSTRAINTS
    for ( i=partList.begin(); i!=partList.end(); ++i ) {                                    // for each body part
        if ( !(*i)->evaluateConstraints() ) {                                               // if the constraints are not satisfied
			for (m = (*i)->begin(); m!=(*i)->end(); ++m) {                                  // set all primitives' collision status 
				for (j = (*m)->begin(); j!=(*m)->end(); ++j) {
                    for (k = (*j)->primitives.begin(); k!=(*j)->primitives.end(); ++k) {
                        (*k)->setColliding(CONSTRAINT);
                    }
					//setColliding((*j));
				}
			}
			addReflexCollision();
		}
    }
}


void Robot::publishState()
{
	emit collisions(numCollisions);
	numCollisions = 0;
	
	emit reflexCollisions( numReflexCollisions );
	numReflexCollisions = 0;
	
    QVector<BodyPart*>::iterator i;
    for ( i=partList.begin(); i!=partList.end(); ++i )
        (*i)->publishState();
    
	//RobotObservation obs = observe();
	//emit observation(obs);
	
	//processState();
}

/**********************
 ***	GET STUFF	***
 **********************/
bool Robot::partIdxInRange( int idx ) const {
    if ( idx >= 0 && idx < partList.size() ) { return 1; }
    else { return 0; }
}

bool Robot::motorIdxInRange( int partNum, int idx ) const {
    if ( idx >= 0 && idx < partList.at(partNum)->size() ) { return 1; }
    else { return 0; }
}

int Robot::getNumMotors( int partNum ) const
{
    if ( partIdxInRange(partNum) ) { return partList.at(partNum)->size(); }
    else { return 0; }
}

const QString* Robot::getPartName( int partNum ) const
{
    if ( partIdxInRange(partNum) ) { return &partList.at(partNum)->name(); }
    else { return 0; }
}

const QString* Robot::getMotorName( int partNum, int motorNum ) const
{
    if ( partIdxInRange(partNum) && motorIdxInRange(partNum,motorNum) ) {
	return &partList.at(partNum)->at(motorNum)->name(); }
    else { return 0; }
}

BodyPart* Robot::getPartByName(const QString &partName)
{
    QVector<BodyPart*>::iterator i;
    for ( i=partList.begin(); i!=partList.end(); ++i ) {
        if ( (*i)->name() == partName )
		{
			return (*i);
		}
    }
    return 0;
}

Motor* Robot::getMotorByName(const QString &motorName)
{
    QVector<Motor*>::iterator i;
    for ( i=motorList.begin(); i!=motorList.end(); ++i ) {
        if ( (*i) && (*i)->name() == motorName )
		{ 
			return (*i);
		}
    }
    return 0;
}


/*void Robot::appendTreeToModel( KinTreeNode* node )
{
	QVector<KinTreeNode*>::iterator i;
	if ( node == NULL ) {
		for ( i=tree.begin(); i!=tree.end(); ++i )
			appendTreeToModel(*i);
	} else {
		for ( i=node->children.begin(); i!=node->children.end(); ++i ) {
			appendTreeToModel(*i);
		}
		model->appendObject(node);
	}
}*/

void Robot::appendMarkersToModel()
{
	//printf("APPENDING MARKERS TO MODEL\n");
	QVector<Marker*>::iterator i;
	for ( i=markers.begin(); i!=markers.end(); ++i ) {
			model->appendObject( (*i)->getTracerObject() );
            model->appendObject( (*i)->getNormalObject() );
	}
    //model->appendObject(<#KinematicModel::KinTreeNode *#>)
}

int Robot::getNumPrimitives()
{
	//printf("Robot.getNumPrimitives()\n");
	int result = 0;
	QVector<KinTreeNode*>::iterator i;
    for ( i=tree.begin(); i!=tree.end(); ++i ) {
        result += (*i)->getNumPrimitives();
    }
    QVector<Marker*>::iterator j;
    for ( j=markers.begin(); j!=markers.end(); ++j ) {
        result += (*j)->getTracerObject()->getPrimitives().size();
        result += (*j)->getTracerObject()->getFieldPrimitives().size();
    }
	//printf(" num primitives: %d\n", result );
	return result;
}


/*bool Robot::isColliding() const
{
	QVector<KinTreeNode*>::const_iterator i;
    for ( i=tree.begin(); i!=tree.end(); ++i )
	{
        if ( (*i)->isColliding() )
		{
			return true;
		}
    }
	return false;
}*/

/*
void Robot::printLinks()
{
    QVector<KinTreeNode*>::iterator i;
    for ( i=tree.begin(); i!=tree.end(); ++i ) {
        (*i)->printAll();
    }
}
void Robot::printBodyParts()
{
    QVector<BodyPart*>::iterator i;
    QVector<Motor*>::iterator j;
    for ( i=partList.begin(); i!=partList.end(); ++i )
	{
		printf("**********************************************\n");
        printf(" BodyPart: %s - %d motors\n", (*i)->name().toStdString().c_str(), (*i)->size());
		printf("**********************************************\n");
		
        for ( j=(*i)->begin(); j!=(*i)->end(); ++j )
		{
			printf("    ======================================\n");
			printf("      motor: %s\n", (*j)->name().toStdString().c_str());
			printf("    ======================================\n");
            (*j)->print();
        }
		printf("**********************************************\n");
    }
}
*/
