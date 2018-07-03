#include "BulletPhysics.h"

namespace bpa
{
BulletPhysics::BulletPhysics()
  : defaultContactProcessingThreshold_(BT_LARGE_FLOAT)
  , broadphase_(new btDbvtBroadphase())
  , collisionConfiguration_(new btDefaultCollisionConfiguration())
  , dispatcher_(new btCollisionDispatcher(collisionConfiguration_))
  , solver_(new btSequentialImpulseConstraintSolver())
  , dynamicsWorld_(new btDiscreteDynamicsWorld(dispatcher_, broadphase_, solver_, collisionConfiguration_))

{
  dynamicsWorld_->setGravity(btVector3(0, 0, -10));
}

BulletPhysics::~BulletPhysics()
{
  // cleanup in the reverse order of creation/initialization
  // remove the rigidbodies from the dynamics world and delete them
  for (int i = dynamicsWorld_->getNumCollisionObjects() - 1; i >= 0; i--)
  {
    btCollisionObject* obj = dynamicsWorld_->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState())
    {
      delete body->getMotionState();
    }
    dynamicsWorld_->removeCollisionObject(obj);
    delete obj;
  }
  collisionBodies_.clear();

  // delete convex shapes
  for (int j = 0; j < convexShapes_.size(); j++)
  {
    btConvexShape* shape = convexShapes_[j];
    delete shape;
  }
  convexShapes_.clear();

  delete dynamicsWorld_;
  delete solver_;
  delete dispatcher_;
  delete collisionConfiguration_;
  delete broadphase_;
}
}