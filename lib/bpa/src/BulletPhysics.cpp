#include "BulletPhysics.h"

namespace bpa
{
BulletPhysics* BulletPhysics::instance_ = nullptr;

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

BulletPhysics* BulletPhysics::instance()
{
  if (!instance_)
  {
    return new BulletPhysics();
  }
  return instance_;
}

void BulletPhysics::addBox(const bpa::Box& box, bool rotate)
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
  addBox(mass, size, origin);
}

bool BulletPhysics::isColliding(const bpa::Box& new_box) const
{
  // if box and other boxes are touch contact, should not be consided as collided !!!
  if (dynamicsWorld_)
  {
    btScalar mass(0.0);
    btVector3 size(new_box.m_length, new_box.m_width, new_box.m_height);
    btVector3 origin(new_box.position.position(0) + new_box.center_of_mass.position(0),
                     new_box.position.position(1) + new_box.center_of_mass.position(1),
                     new_box.position.position(2) + new_box.center_of_mass.position(2));
    btCollisionShape* colShape = new btBoxShape(btVector3(size.getX() / 2.0, size.getY() / 2.0, size.getZ() / 2.0));
    btRigidBody* body = createRigidBody(mass, colShape, origin);

    ContactResultCallback resultCallback;

#pragma omp critical
    {
      dynamicsWorld_->contactTest(body, resultCallback);
    }

    delete body->getCollisionShape();
    delete body->getMotionState();
    delete body;

    // check all contact distances, and exclude the surface contact points
    double maxDistance = 0.0;
    for (size_t i = 0; i < resultCallback.collisionVec.size(); ++i)
    {  // distance are negtive
      if (floatLessThan(resultCallback.distanceVec[i], maxDistance))
      {
        maxDistance = resultCallback.distanceVec[i];
        // std::cout << "collision distance is = " << maxDistance << std::endl;
      }
    }
    if (!resultCallback.collisionVec.empty() && (std::fabs(maxDistance) > std::numeric_limits<float>::epsilon()))
      return true;
  }
  return false;
}

bool BulletPhysics::isColliding(const bpa::Box& new_box, const bpa::Box& old_box) const
{
  // if box and other boxes are touch contact, should not be consided as collided !!!
  if (dynamicsWorld_)
  {
    // new box
    Eigen::Vector3d bbox, old_bbox;
    if (new_box.is_rotated)
      bbox << new_box.m_width, new_box.m_length, new_box.m_height;
    else
      bbox << new_box.m_length, new_box.m_width, new_box.m_height;
    btScalar mass(0.0);
    btVector3 size(btVector3(bbox(0) - 0.01, bbox(1) - 0.01, bbox(2) - 0.01));
    btVector3 origin(new_box.position.position(0) + bbox(0) / 2.0, new_box.position.position(1) + bbox(1) / 2.0,
                     new_box.position.position(2) + bbox(2) / 2.0);
    btCollisionShape* colShape = new btBoxShape(btVector3(size.getX() / 2.0, size.getY() / 2.0, size.getZ() / 2.0));
    btRigidBody* body = createRigidBody(mass, colShape, origin);

    // old box
    if (old_box.is_rotated)
      old_bbox << old_box.m_width, old_box.m_length, old_box.m_height;
    else
      old_bbox << old_box.m_length, old_box.m_width, old_box.m_height;
    btVector3 sizeOld(btVector3(old_bbox(0), old_bbox(1), old_bbox(2)));
    btVector3 originOld(old_box.position.position(0) + old_bbox(0) / 2.0,
                        old_box.position.position(1) + old_bbox(1) / 2.0,
                        old_box.position.position(2) + old_bbox(2) / 2.0);
    btCollisionShape* shapeOld =
        new btBoxShape(btVector3(sizeOld.getX() / 2.0, sizeOld.getY() / 2.0, sizeOld.getZ() / 2.0));
    btRigidBody* bodyOld = createRigidBody(mass, shapeOld, originOld);

    // checking collistion
    ContactResultCallback resultCallback;

#pragma omp critical
    {
      dynamicsWorld_->contactPairTest(body, bodyOld, resultCallback);
    }

    delete body->getCollisionShape();
    delete body->getMotionState();
    delete body;

    delete bodyOld->getCollisionShape();
    delete bodyOld->getMotionState();
    delete bodyOld;

    // check all contact distances, and exclude the surface contact points
    double maxDistance = 0.0;
    for (size_t i = 0; i < resultCallback.collisionVec.size(); ++i)
    {
      if (floatLessThan(resultCallback.distanceVec[i], maxDistance))
      {
        maxDistance = resultCallback.distanceVec[i];
        // std::cout << "collision distance is = " << maxDistance << std::endl;
      }
    }
    if (!resultCallback.collisionVec.empty() && std::fabs(maxDistance) > std::numeric_limits<float>::epsilon())
      return true;
  }
  return false;
}

int BulletPhysics::numCollisionObjects() const
{
  return dynamicsWorld_->getNumCollisionObjects();
}

void BulletPhysics::addBoxes(const std::vector<bpa::Box>& boxes)
{
  for (const bpa::Box& b : boxes)
  {
    addBox(b);
  }
}

btRigidBody* BulletPhysics::createRigidBody(btScalar mass, btCollisionShape* shape, btVector3 origin) const
{
  btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
  shape->setMargin(0.0);

  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(origin.getX(), origin.getY(), origin.getZ()));

  bool isDynamic = (mass != 0.f);

  btVector3 localInertia(0, 0, 0);
  if (isDynamic)
    shape->calculateLocalInertia(mass, localInertia);

  btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
  btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
  btRigidBody* body = new btRigidBody(cInfo);
  body->setContactProcessingThreshold(defaultContactProcessingThreshold_);

  return body;
}

void BulletPhysics::addBox(btScalar mass, btVector3 size, btVector3 origin)
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

BulletPhysics::ContactResultCallback::ContactResultCallback()
  : collision(false)
  , distance(0)
  , positionWorldOnA(0.0, 0.0, 0.0)
  , positionWorldOnB(0.0, 0.0, 0.0)
  , localPointA(0.0, 0.0, 0.0)
  , localPointB(0.0, 0.0, 0.0)
  , normalWorldOnB(0.0, 0.0, 0.0)
{
}

btScalar BulletPhysics::ContactResultCallback::addSingleResult(btManifoldPoint& cp,
                                                               const ::btCollisionObjectWrapper* colObj0, int partId0,
                                                               int index0, const btCollisionObjectWrapper* colObj1,
                                                               int partId1, int index1)
{
  collision = true;
  distance = cp.getDistance();
  positionWorldOnA = cp.getPositionWorldOnA();
  positionWorldOnB = cp.getPositionWorldOnB();
  normalWorldOnB = cp.m_normalWorldOnB;
  localPointA = cp.m_localPointA;
  localPointB = cp.m_localPointB;

  collisionVec.push_back(collision);
  distanceVec.push_back(distance);
  posWorldOnAVec.push_back(positionWorldOnA);
  posWorldOnBVec.push_back(positionWorldOnB);
  normalWorldOnBVec.push_back(normalWorldOnB);

  return 0;
}

void BulletPhysics::ContactResultCallback::clearData()
{
  collision = false;
  distance = 0;
  positionWorldOnA.setValue(0.0, 0.0, 0.0);
  positionWorldOnB.setValue(0.0, 0.0, 0.0);
  normalWorldOnB.setValue(0.0, 0.0, 0.0);
  localPointA.setValue(0.0, 0.0, 0.0);
  localPointB.setValue(0.0, 0.0, 0.0);

  collisionVec.clear();
  distanceVec.clear();
  posWorldOnAVec.clear();
  posWorldOnBVec.clear();
  normalWorldOnBVec.clear();
}
}