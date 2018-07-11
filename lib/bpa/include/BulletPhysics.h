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

  bool isColliding(const bpa::Box& box) const;
  bool isColliding(const bpa::Box& box_a, const bpa::Box& box_b) const;
  bool isPointContact(const Eigen::Vector3d& point) const;

  Eigen::Vector3d castRays(const Eigen::Vector3d& point, const Eigen::Vector3d& direction) const;

  // assume old_box is already in physics world
  // add new_box into the world, calculate the support area, and remove it from the world
  double getSupportArea(const bpa::Box& new_box, const bpa::Box& old_box);

  int numCollisionObjects() const;

private:
  void addBox(btScalar mass, btVector3 size, btVector3 origin);
  btRigidBody* createRigidBody(btScalar mass, btCollisionShape* shape, btVector3 origin) const;
  btRigidBody* createPointSphere(btVector3 origin) const;

  // assuming there are 4 points and they form a parallelogram on a horizontal plane
  double getHorizontalArea(std::vector<btVector3>& points) const;

  static BulletPhysics* instance_;

  btScalar defaultContactProcessingThreshold_;
  btBroadphaseInterface* broadphase_;
  btDefaultCollisionConfiguration* collisionConfiguration_;
  btCollisionDispatcher* dispatcher_;
  btConstraintSolver* solver_;
  btDynamicsWorld* dynamicsWorld_;
  btAlignedObjectArray<btRigidBody*> collisionBodies_;
  std::vector<btConvexShape*> convexShapes_;

  struct ContactResultCallback : public btCollisionWorld::ContactResultCallback
  {
    ContactResultCallback();

    btScalar addSingleResult(btManifoldPoint& cp, const ::btCollisionObjectWrapper* colObj0, int partId0, int index0,
                             const btCollisionObjectWrapper* colObj1, int partId1, int index1);

    void clearData();

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
};
}  // namespace bpa

#endif  // BULLET_PHYSICS_H