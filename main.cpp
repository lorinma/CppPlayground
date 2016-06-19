#include <stdio.h>
#include <iostream>
using namespace std;
#include "client_http.hpp"
typedef SimpleWeb::Client<SimpleWeb::HTTP> HttpClient;
#include "rapidjson/document.h"
using namespace rapidjson;
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

int main(int argc, char** argv)
{
    HttpClient client("seebim-api-lorinma.c9users.io");
    auto r1=client.request("GET", "/geometry");
    std::ostringstream oss;
    oss << r1->content.rdbuf();
    std::string str=oss.str();
    const char* json=str.c_str();
    Document document;
    document.Parse(json);
    const Value& items = document["_items"];
    
    cout<<"number of shapes: "<<items.Size()<<endl;
    
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    	
	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

    //Bullet Collision Detection can also be used without the Dynamics/Extras.
    btCollisionWorld* world = new btCollisionWorld(dispatcher,overlappingPairCache,collisionConfiguration);

	//keep tracj of the shapes, we release memory at exit.
	//maje sure to re-use collision shapes among rigid bodies whenever possible!
    btAlignedObjectArray<btCollisionShape*> collisionShapes;

    for (SizeType i = 0; i < items.Size(); i++){
        cout<<"guid: " << items[i]["GlobalId"].GetString()<<endl;
        // get all the vertices from seebim-api
        const Value& Vertices=items[i]["Geometry"]["Vertices"];
        double V_array [Vertices.Size()][3];
        for (SizeType j = 0; j < Vertices.Size(); j++){ // Uses SizeType instead of size_t
            for (int v=0; v<3; v++){
                // change units from mm (seebim-api default) to m, this is bullet's requirement in margin setting
                V_array[j][v]=Vertices[j][v].GetDouble()/1000;
            }
        }
        // get all the faces'indices from seebim-api
        const Value& Faces=items[i]["Geometry"]["Faces"];
        int Face_Size = Faces.Size();
        int F_array [Face_Size][3];
        for (SizeType j = 0; j < Faces.Size(); j++){
            for (int f =0; f<3; f++){
                F_array[j][f]=Faces[j][f].GetInt();
            }
        }
        // create mesh in bullet3
        btTriangleMesh* mesh = new btTriangleMesh();
        for(int f=0;f<Face_Size;f++){
            btVector3 quad[3];
            for(int v=0;v<3;v++){
                quad[v]=btVector3(V_array[F_array[f][v]][0],V_array[F_array[f][v]][1],V_array[F_array[f][v]][2]);
            }
            mesh->addTriangle(quad[0],quad[1],quad[2],true);
        }
        btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(mesh,true,true);
        btCollisionObject* o = new btCollisionObject();
        o->setCollisionShape(trimesh);
        world->addCollisionObject(o);
    }
    // this can perform broadphase collision detection?
    world->performDiscreteCollisionDetection();
    cout<<world->getCollisionObjectArray().size()<<endl;

    struct MyContactResultCallback : public btCollisionWorld::ContactResultCallback{
		bool m_connected;
		btScalar m_margin;
		MyContactResultCallback() :m_connected(false),m_margin(0.05){
		}
		virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1){
			if (cp.getDistance()<=m_margin)
			    m_connected = true;
			return 1.f;
		}
	};
			   
    MyContactResultCallback result;
    world->contactPairTest(world->getCollisionObjectArray()[9],world->getCollisionObjectArray()[15],result);
	if (result.m_connected){
	    cout<<"connected"<<endl;
// 		btConnection tmp;
// 		tmp.m_childIndex0 = i;
// 		tmp.m_childIndex1 = j;
// 		tmp.m_childShape0 = compound->getChildShape(i);
// 		tmp.m_childShape1 = compound->getChildShape(j);
// 		tmp.m_strength = 1.f;//??
// 		m_connections.push_back(tmp);
	}
	else{
	    cout<<"no touch"<<endl;
	}
    // world->getCollisionObjectArray();
    // collisionShapes = world->getCollisionObjectArray();
    
    //delete world
	delete world;

// 	//delete solver
// 	delete solver;

	//delete broadphase
	delete overlappingPairCache;

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();
}
