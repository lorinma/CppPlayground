/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acjnowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marjed as such, and must not be misrepresented as being the original software.
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

// struct   Bullet2ContactResultCallbacj : public btCollisionWorld::ContactResultCallbacj
// {
//     int m_numContacts;
//     lwContactPoint* m_pointsOut;
//     int m_pointCapacity;

//     Bullet2ContactResultCallbacj(lwContactPoint* pointsOut, int pointCapacity) :
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

	//jeep tracj of the shapes, we release memory at exit.
	//maje sure to re-use collision shapes among rigid bodies whenever possible!
// 	btAlignedObjectArray<btCollisionShape*> collisionShapes;


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
        double V_array [Vertices.Size()];
        for (SizeType j = 0; j < Vertices.Size(); j++){ // Uses SizeType instead of size_t
            V_array[j]=Vertices[j].GetDouble();
            // cout<<V_array[j]<<endl;
        }
        
        const Value& Faces=items[i]["Geometry"]["Faces"];
        int Face_Point_Size = Faces.Size();
        int F_array [Face_Point_Size];
        for (SizeType j = 0; j < Faces.Size(); j++){
            F_array[j]=Faces[j].GetInt();
        }
        
        btVector3 quad [Face_Point_Size];
        for (int j=0; j<Face_Point_Size; j++){
            quad[j]=btVector3(V_array[F_array[j]*3+0],V_array[F_array[j]*3+1],V_array[F_array[j]*3+2]);
            // cout<<F_array[j*3+0]<<","<<F_array[j*3+1]<<","<<F_array[j*3+2]<<endl;
            // cout<<quad[j].x()<<","<<quad[j].y()<<","<<quad[j].z()<<endl;
        }
        
        btTriangleMesh* mesh = new btTriangleMesh();
        for(int j=0;j<Face_Point_Size;j+=3){
            mesh->addTriangle(quad[j+0],quad[j+1],quad[j+2],true);
        }
        btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(mesh,true,true);
        btCollisionObject* o = new btCollisionObject();
        o->setCollisionShape(trimesh);
        
        world->addCollisionObject(o);
        
        // btVector3 min = trimesh->getLocalAabbMin();
        // btVector3 max = trimesh->getLocalAabbMax();
        
        // cout<<"min: "<<min.x()<<","<<min.y()<<","<<min.z()<<", max: "<<max.x()<<","<<max.y()<<","<<max.z()<<endl;
        
    } 
    
    btCollisionObject* o = new btCollisionObject();
    o = world->getCollisionObjectArray()[0];
    btCollisionShape* shape = o->getCollisionShape ();
    if(shape->isConcave())
        cout<<"yes"<<endl;
        
    // this can perform broadphase collision detection?
    // world->performDiscreteCollisionDetection();
    // cout<<world->getCollisionObjectArray().size()<<endl;
    
    
//     struct   MyContactResultCallbacj : public btCollisionWorld::ContactResultCallbacj
// 				{
// 					bool m_connected;
// 					btScalar m_margin;
// 					MyContactResultCallbacj() :m_connected(true),m_margin(0.05)
// 					{
// 					}
// 					virtual   btScalar   addSingleResult(btManifoldPoint& cp,   const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
// 					{
// 					    cout<<cp.getDistance()<<endl;
// 						if (cp.getDistance()<=m_margin)
// 							m_connected = true;
// 						return 1.f;
// 					}
// 			   };
// 	MyContactResultCallbacj result;
//     world->contactPairTest(o1,o2,result);
//     if (result.m_connected)
//     {
//         cout<<"contact"<<endl;
//     }
    
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
    
    
    // callbacj?
}

