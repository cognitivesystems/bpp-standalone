#ifndef PHYSICS_BULLET_H
#define PHYSICS_BULLET_H

/// btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <LinearMath/btAlignedObjectArray.h>
#include "Box.h"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class btCollisionWorld;

// RaytestDemo shows how to use the btCollisionWorld::rayTest feature

class PhysicsBullet
{
public:
  PhysicsBullet();

  virtual ~PhysicsBullet()
  {
    exitPhysics();
  }

  void addNewBoxToPhysics(bpa::Box& new_box);
  void addNewBoxToPhysicsNoRot(bpa::Box& new_box);
  void addNewBoxesToPhysics(std::vector<bpa::Box>& packed_boxes);

  bool isColliding(bpa::Box& new_box);

  // TODO: implement the following in BulletPhysics class

  Eigen::Vector3d castRays(Eigen::Vector3d& point, Eigen::Vector3d& direction);

  bool isCollidingBox(bpa::Box& new_box, bpa::Box& old_box);
  bool isPointContact(Eigen::Vector3d& point);

  // only get the support area (down)
  double getSupportArea(bpa::Box& new_box, bpa::Box& old_box);
  // get all side contact areas
  double getContactArea(bpa::Box& new_box, bpa::Box& old_box);

  double getMinimumDistance(bpa::Box& new_box);
  double getMinimumDistance(bpa::Box& new_box, bpa::Box& old_box);

  void addBinBoundingBox();

  btAlignedObjectArray<btRigidBody*> m_collisionBodies;

private:
  //    btAlignedObjectArray<btRigidBody*>	m_collisionBodies;
  btDynamicsWorld* m_dynamicsWorld;
  btScalar m_defaultContactProcessingThreshold;

  // keep the collision shapes, for deletion/cleanup
  std::vector<btConvexShape*> m_convexShapes;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btConstraintSolver* m_solver;
  btDefaultCollisionConfiguration* m_collisionConfiguration;

  struct ContactResultCallback : public btCollisionWorld::ContactResultCallback
  {
    ContactResultCallback();

    btScalar addSingleResult(btManifoldPoint& cp, const ::btCollisionObjectWrapper* colObj0, int partId0, int index0,
                             const btCollisionObjectWrapper* colObj1, int partId1,
                             int index1);  // define your code here!

    void clearDatas();

    bool collision;
    btScalar distance;
    btVector3 positionWorldOnA;
    btVector3 positionWorldOnB;

    btVector3 localPointA;
    btVector3 localPointB;
    btVector3 normalWorldOnB;

    std::vector<bool> collisionVec;
    std::vector<btScalar> distanceVec;
    std::vector<btVector3> posWorldOnAVec;
    std::vector<btVector3> posWorldOnBVec;
    std::vector<btVector3> normalWorldOnBVec;
  };

  void initPhysics();
  void exitPhysics();

  void localCreateRigidBox(btScalar mass, btVector3 size, btVector3 origin);
  btRigidBody* localCreateRigidBody(btScalar mass, btCollisionShape* shape, btVector3 origin);
  btRigidBody* localCreateRigidBodyRot(btScalar mass, btCollisionShape* shape, btVector3 origin, btQuaternion q);

  btRigidBody* localCreatePointSphere(btVector3 origin);
  bool isBoxContact(bpa::Box new_box, bpa::Box& old_box, ContactResultCallback& resultCallback);
};

#endif  // PHYSICS_BULLET_H
