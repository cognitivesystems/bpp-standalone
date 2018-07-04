#ifndef BULLET_PHYSICS_H
#define BULLET_PHYSICS_H

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <vector>
#include "Box.h"

namespace bpa
{
class BulletPhysics
{
public:
  BulletPhysics();
  ~BulletPhysics();

  static BulletPhysics* instance();

  void addBox(const bpa::Box& box, bool rotate = true);
  void addBoxes(const std::vector<bpa::Box>& boxes);

private:
  void addBox(btScalar mass, btVector3 size, btVector3 origin);

  static BulletPhysics* instance_;

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