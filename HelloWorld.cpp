/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///-----includes_start-----
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include <stdio.h>
#include <iostream>
using namespace std;

#include "client_http.hpp"

//Added for the json-example
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "rapidjson/document.h"
using namespace rapidjson;
//Added for the default_resource example
#include <boost/filesystem.hpp>

using namespace std;
//Added for the json-example:
using namespace boost::property_tree;

typedef SimpleWeb::Client<SimpleWeb::HTTP> HttpClient;

/// This is a Hello World program for running a basic Bullet physics simulation

// struct   Bullet2ContactResultCallback : public btCollisionWorld::ContactResultCallback
// {
//     int m_numContacts;
//     lwContactPoint* m_pointsOut;
//     int m_pointCapacity;

//     Bullet2ContactResultCallback(lwContactPoint* pointsOut, int pointCapacity) :
//             m_numContacts(0),
//             m_pointsOut(pointsOut),
//             m_pointCapacity(pointCapacity)
//     {
//     }
//     virtual   btScalar   addSingleResult(btManifoldPoint& cp,   const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
//     {
//         if (m_numContacts<m_pointCapacity)
//         {
//             lwContactPoint& ptOut = m_pointsOut[m_numContacts];
//             ptOut.m_distance = cp.m_distance1;
//             ptOut.m_normalOnB[0] = cp.m_normalWorldOnB.getX();
//             ptOut.m_normalOnB[1] = cp.m_normalWorldOnB.getY();
//             ptOut.m_normalOnB[2] = cp.m_normalWorldOnB.getZ();
//             ptOut.m_ptOnAWorld[0] = cp.m_positionWorldOnA[0];
//             ptOut.m_ptOnAWorld[1] = cp.m_positionWorldOnA[1];
//             ptOut.m_ptOnAWorld[2] = cp.m_positionWorldOnA[2];
//             ptOut.m_ptOnBWorld[0] = cp.m_positionWorldOnB[0];
//             ptOut.m_ptOnBWorld[1] = cp.m_positionWorldOnB[1];
//             ptOut.m_ptOnBWorld[2] = cp.m_positionWorldOnB[2];
//             m_numContacts++;
//         }

//         return 1.f;
//     }
// };

int main(int argc, char** argv)
{
	//Client examples
    HttpClient client("restfulifc-lorinma.c9users.io");
    auto r1=client.request("GET", "/entity/element");
    std::ostringstream oss;
    oss << r1->content.rdbuf();
    std::string str=oss.str();
    const char* json=str.c_str();

    Document document;
    document.Parse(json);
    const Value& items = document["_items"];
    for (SizeType i = 0; i < items.Size(); i++){
        const Value& Vertices=items[i]["Geometry"]["Vertices"];
        for (SizeType j = 0; j < Vertices.Size(); j++){
            cout<<Vertices[j].GetDouble()<<endl;
        }
        const Value& Faces=items[i]["Geometry"]["Faces"];
        for (SizeType j = 0; j < Faces.Size(); j++){
            cout<<Faces[j].GetInt()<<endl;
        }
        // Uses SizeType instead of size_t
    } // Uses SizeType instead of size_t
    
	///-----includes_end-----

	// int i;
	///-----initialization_start-----

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

//	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
//	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

//    Bullet Collision Detection can also be used without the Dynamics/Extras.
    btCollisionWorld* world = new btCollisionWorld(dispatcher,overlappingPairCache,collisionConfiguration);

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
// 	btAlignedObjectArray<btCollisionShape*> collisionShapes;


    btVector3 quad1[] = {
            btVector3(2445,   0,1900),
            btVector3(2445, 200,1900),
            btVector3(2445, 200,-200),
            btVector3(2445,   0,1900),
            btVector3(2445, 200,-200),
            btVector3(2445,   0,-200),
            btVector3(2445,   0,1900),
            btVector3(2445,   0,-200),
            btVector3(2545,   0,-200),
            btVector3(2545,   0,1900),
            btVector3(2445,   0,1900),
            btVector3(2545,   0,-200),
            btVector3(2545, 200,1900),
            btVector3(2545,   0,1900),
            btVector3(2545,   0,-200),
            btVector3(2545, 200,1900),
            btVector3(2545,   0,-200),
            btVector3(2545, 200,-200),
            btVector3(2545, 200,1900),
            btVector3(2545, 200,-200),
            btVector3(2445, 200,-200),
            btVector3(2445, 200,1900),
            btVector3(2545, 200,1900),
            btVector3(2445, 200,-200),
            btVector3(2545, 200,-200),
            btVector3(2545,   0,-200),
            btVector3(2445,   0,-200),
            btVector3(2445, 200,-200),
            btVector3(2545, 200,-200),
            btVector3(2445,   0,-200),
            btVector3(2445,   0,1900),
            btVector3(2545,   0,1900),
            btVector3(2545, 200,1900),
            btVector3(2445,   0,1900),
            btVector3(2545, 200,1900),
            btVector3(2445, 200,1900)};
    btTriangleMesh* mesh1 = new btTriangleMesh();
    int len1=(sizeof(quad1)/sizeof(*quad1));
    for(int i=0;i<len1;i+=3){
        mesh1->addTriangle(quad1[i+0],quad1[i+1],quad1[i+2],true);
    }
    btBvhTriangleMeshShape* trimesh1 = new btBvhTriangleMeshShape(mesh1,true,true);
    btCollisionObject* o1 = new btCollisionObject();
    o1->setCollisionShape(trimesh1);
    
    btVector3 quad2[] = {
            btVector3(2845, 50,1600),
            btVector3(2545, 50,1600),
            btVector3(2545, 50,1400),
            btVector3(2845, 50,1400),
            btVector3(2845, 50,1600),
            btVector3(2545, 50,1400),
            btVector3(2845, 50,1400),
            btVector3(2545, 50,1400),
            btVector3(2545,150,1400),
            btVector3(2845,150,1400),
            btVector3(2845, 50,1400),
            btVector3(2545,150,1400),
            btVector3(2845,150,1400),
            btVector3(2545,150,1400),
            btVector3(2545,150,1600),
            btVector3(2845,150,1600),
            btVector3(2845,150,1400),
            btVector3(2545,150,1600),
            btVector3(2845,150,1600),
            btVector3(2545,150,1600),
            btVector3(2545, 50,1600),
            btVector3(2845, 50,1600),
            btVector3(2845,150,1600),
            btVector3(2545, 50,1600),
            btVector3(2545,150,1600),
            btVector3(2545, 50,1400),
            btVector3(2545, 50,1600),
            btVector3(2545,150,1600),
            btVector3(2545,150,1400),
            btVector3(2545, 50,1400),
            btVector3(2845, 50,1600),
            btVector3(2845, 50,1400),
            btVector3(2845,150,1600),
            btVector3(2845, 50,1400),
            btVector3(2845,150,1400),
            btVector3(2845,150,1600),
            btVector3(2445, 50,1600),
            btVector3(   0, 50,1600),
            btVector3(   0, 50,1400),
            btVector3(2445, 50,1400),
            btVector3(2445, 50,1600),
            btVector3(   0, 50,1400),
            btVector3(2445,150,1400),
            btVector3(2445, 50,1400),
            btVector3(   0, 50,1400),
            btVector3(2445,150,1400),
            btVector3(   0, 50,1400),
            btVector3(   0,150,1400),
            btVector3(2445,150,1400),
            btVector3(   0,150,1400),
            btVector3(   0,150,1600),
            btVector3(2445,150,1600),
            btVector3(2445,150,1400),
            btVector3(   0,150,1600),
            btVector3(2445, 50,1600),
            btVector3(2445,150,1600),
            btVector3(   0,150,1600),
            btVector3(2445, 50,1600),
            btVector3(   0,150,1600),
            btVector3(   0, 50,1600),
            btVector3(   0,150,1400),
            btVector3(   0, 50,1400),
            btVector3(   0, 50,1600),
            btVector3(   0,150,1400),
            btVector3(   0, 50,1600),
            btVector3(   0,150,1600),
            btVector3(2445, 50,1600),
            btVector3(2445, 50,1400),
            btVector3(2445,150,1400),
            btVector3(2445,150,1600),
            btVector3(2445, 50,1600),
            btVector3(2445,150,1400)};
    btTriangleMesh* mesh2 = new btTriangleMesh();
    int len2=(sizeof(quad2)/sizeof(*quad2));
    for(int i=0;i<len2;i+=3){
        mesh2->addTriangle(quad2[i+0],quad2[i+1],quad2[i+2],true);
    }
    btBvhTriangleMeshShape* trimesh2 = new btBvhTriangleMeshShape(mesh2,true,true);
    btCollisionObject* o2 = new btCollisionObject();
    o2->setCollisionShape(trimesh2);
    
    struct   MyContactResultCallback : public btCollisionWorld::ContactResultCallback
				{
					bool m_connected;
					btScalar m_margin;
					MyContactResultCallback() :m_connected(true),m_margin(0.05)
					{
					}
					virtual   btScalar   addSingleResult(btManifoldPoint& cp,   const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
					{
					    cout<<cp.getDistance()<<endl;
						if (cp.getDistance()<=m_margin)
							m_connected = true;
						return 1.f;
					}
			   };
	MyContactResultCallback result;
    world->contactPairTest(o1,o2,result);
    if (result.m_connected)
    {
        cout<<"contact"<<endl;
    }
    
    // btTransform t;
    // btVector3 min,max,o;
    // // trimesh1->getAabb(t,min,max);
    // min=trimesh2->getLocalAabbMin();
    // min=trimesh2->getLocalAabbMax();
    // // o=t.getOrigin();
    // // cout<<trimesh1->isConvex()<<endl;
    // // cout<<o.x()<<","<<o.y()<<","<<o.z()<<endl;
    // cout<<min.x()<<","<<min.y()<<","<<min.z()<<endl;
    // cout<<max.x()<<","<<max.y()<<","<<max.z()<<endl;
    
    
    // callback?
}

