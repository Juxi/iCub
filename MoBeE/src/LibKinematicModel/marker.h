/*******************************************************************
 ***  Copyright (C) 2011-2015 Mikhail Frank, Juxi Leitner        ***
 ***  CopyPolicy: Released under the terms of the GNU GPL v2.0.  ***
 ******************************************************************/

#ifndef MARKER_H
#define MARKER_H

#include "sphere.h"
#include "cylinder.h"
namespace KinematicModel {


class CompositeObject;
class Motor;


class Marker
{
public:
	Marker(KinTreeNode* object, QString name, QVector3D n) : m_object(object),
                                                            normal(QVector4D(n,1)),
                                                            tracerObject(NULL),
                                                            m_name(name),
                                                            d(0)
	{
	}

	inline KinTreeNode* node()
	{ return m_object; }
    
    //inline const QVector3D getNorm() const
    //{ return (m_object->getT()*normal).toVector3D(); }
    inline const QVector3D getNorm() const
    { 
        QMatrix4x4 T = m_object->getT();
        T(0,3)=0; T(1,3)=0; T(2,3)=0;
        //QVector4D n = T*normal;
        //printf("getting T %s: %f %f %f\n                %f %f %f\n                %f %f %f\n",m_name.toStdString().c_str() ,T(0,0),T(0,1),T(0,2),T(1,0),T(1,1),T(1,2),T(2,0),T(2,1),T(2,2));
        //printf("getting normal %s: %f %f %f\n",m_name.toStdString().c_str() ,n.x(),n.y(),n.z());
        return (T*normal).toVector3D();
    }
    
    
	inline const CompositeObject* tracer() const
	{ return tracerObject; }

	inline QString name() const { return m_name; }
	
	inline CompositeObject* getTracerObject()   { return tracerObject; }
    
    inline CompositeObject* getNormalObject()
	{ return normalObject; }
	
    inline void createNormal( DT_ResponseClass c, QVector3D& h, double r, double len, QColor qc )
    {
        normalObject = new CompositeObject(c,c);
        //normalObject->persistent = true;
        normalObject->setFreeColor(qc);
        
        PrimitiveObject* p = new Cylinder(r,len);
        p->setFreeColor(qc);
        p->setSpecialEulerOrientation(h,0);
        p->translate( h.normalized() * len/2 );
        
        normalObject->appendPrimitive(p);
    }
    
	inline void createTracer( DT_ResponseClass c, int num, double r, QColor qc )
	{
        printf("CREATING TRACER GEOMS - %s\n",m_name.toStdString().c_str());
		d = 2*r;
		tracerObject = new CompositeObject(c,c); 
		//tracerObject->persistent = true;
		tracerObject->setFreeColor(qc);
        
        PrimitiveObject* p;// = new Cylinder(0.01,0.25);
        //p->setFreeColor(qc);
        //p->setSpecialEulerOrientation(normal.toVector3D());
        //QVector3D foo = normal.toVector3D()/4;
        //printf("TRANSLATING - %f %f %f\n",foo.x(),foo.y(),foo.z());
        //p->translate(foo);
        
		//tracerObject->appendPrimitive(p);
        
		int i;
		for (i=0; i<num; i++)
		{
			p = new Sphere(r);
			tracerObject->appendPrimitive(p);
			if ( i > 0 ) qc.setAlphaF( 1 - (double)i/(double)num);
			p->setFreeColor(qc);
		}
		//printf("created %d balls\n",i);
	}
	
	inline void update()
	{
        //const QVector<PrimitiveObject*>& normal = normalObject->data();
        //if (*normal.begin())
        //    (*normal.begin())->setT(m_object->getT());
        
        normalObject->setT(m_object->getT());
        
        
		const QVector<PrimitiveObject*>& tracer = tracerObject->data();
		
		qreal* newT = m_object->getT().data();
		qreal* oldT = (*tracer.begin())->getT().data();
		
        
		//printf("\n");
		//printf("a: %f, %f, %f\n",newT[12],newT[13],newT[14]);
		//printf("b: %f, %f, %f\n",oldT[12],oldT[13],oldT[14]);
		
		qreal	a = newT[12]-oldT[12],
				b = newT[13]-oldT[13],
				c = newT[14]-oldT[14];
		
		double dx = sqrt( a*a + b*b + c*c );
		
		if ( dx > d )
		{
			QVector<PrimitiveObject*>::const_iterator i;
			
			for ( i=tracer.end()-1; i!=tracer.begin(); --i )
				(*i)->setT( (*(i-1))->getT() );
            if (*tracer.begin())
                (*tracer.begin())->setT( m_object->getT() );
		}
    }
    

protected:
	KinTreeNode *m_object;
    QVector4D normal;
    //Transformable M;
    CompositeObject *tracerObject;
    CompositeObject *normalObject;
	QString m_name;
	qreal d;
};


}
#endif
