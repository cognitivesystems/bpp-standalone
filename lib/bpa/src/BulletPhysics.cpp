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
      delete body->getCollisionShape();
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

void BulletPhysics::createBox(const bpa::Box& box, bool rotate)
{
  btScalar mass(0.0);
  btVector3 size;
  btVector3 origin;
  if (rotate && box.is_rotated)
  {
    size = btVector3(box.m_width, box.m_length, box.m_height);
    // COM is the original one, depend on 0 rotation
    origin = btVector3(box.position.position(0) + box.center_of_mass.position(1),
                       box.position.position(1) + box.center_of_mass.position(0),
                       box.position.position(2) + box.center_of_mass.position(2));
  }
  else
  {
    size = btVector3(box.m_length, box.m_width, box.m_height);
    origin = btVector3(box.position.position(0) + box.center_of_mass.position(0),
                       box.position.position(1) + box.center_of_mass.position(1),
                       box.position.position(2) + box.center_of_mass.position(2));
  }
  createBox(mass, size, origin);
}

void BulletPhysics::createBox(btScalar mass, btVector3 size, btVector3 origin)
{
  btCollisionShape* colShape = new btBoxShape(btVector3(size.getX() / 2.0, size.getY() / 2.0, size.getZ() / 2.0));
  colShape->setMargin(-0.00001);  // the margin affect the contact distance and normal.
  btConvexShape* convexShape = new btBoxShape(btVector3(size.getX() / 2.0, size.getY() / 2.0, size.getZ() / 2.0));
  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(origin.getX(), origin.getY(), origin.getZ()));  // center of mass

  // rigidbody is dynamic if and only if mass is non zero, otherwise static
  bool isDynamic = (mass != 0.f);

  btVector3 localInertia(0, 0, 0);
  if (isDynamic)
    colShape->calculateLocalInertia(mass, localInertia);

  // using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
  btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
  btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, colShape, localInertia);
  btRigidBody* body = new btRigidBody(cInfo);
  body->setContactProcessingThreshold(defaultContactProcessingThreshold_);

  body->setRollingFriction(0.05);
  body->setFriction(1);
  dynamicsWorld_->addCollisionObject(body);
  convexShapes_.push_back(convexShape);
  collisionBodies_.push_back(body);

  dynamicsWorld_->performDiscreteCollisionDetection();
  dynamicsWorld_->updateAabbs();
  dynamicsWorld_->computeOverlappingPairs();
}
}