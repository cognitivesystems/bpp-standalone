#ifndef BULLET_PHYSICS_H
#define BULLET_PHYSICS_H

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <vector>
#include "PhysicsEngine.h"

namespace bpa
{
class BulletPhysics : public PhysicsEngine
{
public:
  BulletPhysics();
  virtual ~BulletPhysics();

private:
  virtual void createBox(const bpa::Box& box, bool rotate);
  void createBox(btScalar mass, btVector3 size, btVector3 origin);

  btScalar defaultContactProcessingThreshold_;
  btBroadphaseInterface* broadphase_;
  btDefaultCollisionConfiguration* collisionConfiguration_;
  btCollisionDispatcher* dispatcher_;
  btConstraintSolver* solver_;
  btDynamicsWorld* dynamicsWorld_;
  btAlignedObjectArray<btRigidBody*> collisionBodies_;
  std::vector<btConvexShape*> convexShapes_;
};
}  // namespace engine_factory

#endif  // BULLET_PHYSICS_H