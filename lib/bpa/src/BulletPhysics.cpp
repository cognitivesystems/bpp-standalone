#include <set>
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

bool BulletPhysics::isColliding(const bpa::Box& box) const
{
  // if box and other boxes are touch contact, should not be consided as collided !!!
  if (dynamicsWorld_)
  {
    btScalar mass(0.0);
    btVector3 size(box.m_length, box.m_width, box.m_height);
    btVector3 origin(box.position.position(0) + box.center_of_mass.position(0),
                     box.position.position(1) + box.center_of_mass.position(1),
                     box.position.position(2) + box.center_of_mass.position(2));
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

bool BulletPhysics::isColliding(const bpa::Box& box_a, const bpa::Box& box_b) const
{
  // if box and other boxes are touch contact, should not be consided as collided !!!
  if (dynamicsWorld_)
  {
    // new box
    Eigen::Vector3d bbox, old_bbox;
    if (box_a.is_rotated)
      bbox << box_a.m_width, box_a.m_length, box_a.m_height;
    else
      bbox << box_a.m_length, box_a.m_width, box_a.m_height;
    btScalar mass(0.0);
    btVector3 size(btVector3(bbox(0) - 0.01, bbox(1) - 0.01, bbox(2) - 0.01));
    btVector3 origin(box_a.position.position(0) + bbox(0) / 2.0, box_a.position.position(1) + bbox(1) / 2.0,
                     box_a.position.position(2) + bbox(2) / 2.0);
    btCollisionShape* colShape = new btBoxShape(btVector3(size.getX() / 2.0, size.getY() / 2.0, size.getZ() / 2.0));
    btRigidBody* body = createRigidBody(mass, colShape, origin);

    // old box
    if (box_b.is_rotated)
      old_bbox << box_b.m_width, box_b.m_length, box_b.m_height;
    else
      old_bbox << box_b.m_length, box_b.m_width, box_b.m_height;
    btVector3 sizeOld(btVector3(old_bbox(0), old_bbox(1), old_bbox(2)));
    btVector3 originOld(box_b.position.position(0) + old_bbox(0) / 2.0, box_b.position.position(1) + old_bbox(1) / 2.0,
                        box_b.position.position(2) + old_bbox(2) / 2.0);
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

bool BulletPhysics::isPointContact(const Eigen::Vector3d& point) const
{
  if (dynamicsWorld_)
  {
    btRigidBody* pointBall = createPointSphere(btVector3(point(0), point(1), point(2)));
    // 1: point & world, setMargin = -0.0001
    ContactResultCallback resultCallback;
    dynamicsWorld_->contactTest(pointBall, resultCallback);

    delete pointBall->getCollisionShape();
    delete pointBall->getMotionState();
    delete pointBall;

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

Eigen::Vector3d BulletPhysics::castRays(const Eigen::Vector3d& point, const Eigen::Vector3d& direction) const
{
  Eigen::Vector3d projection;
  projection = point;

  if (dynamicsWorld_)
  {
    // first hit
    btVector3 from = btVector3((float)(point(0)), (float)(point(1)), (float)(point(2)));
    btVector3 to = btVector3((float)(point(0) + 5 * direction(0)), (float)(point(1) + 5 * direction(1)),
                             (float)(point(2) + 5 * direction(2)));

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Problem:  If this point already contacts with a box, then the rayTest will get next point instead of this
    /// contact point.
    /// \brief Solution: check if this direction is contacted. If it is contacted, don't need projection!
    bool needProj = true;
    btRigidBody* pointBall = createPointSphere(from);
    ContactResultCallback resultCallback;
    dynamicsWorld_->contactTest(pointBall, resultCallback);

    delete pointBall->getCollisionShape();
    delete pointBall->getMotionState();
    delete pointBall;

    size_t oldNum = resultCallback.collisionVec.size();

    btRigidBody* pointTemp =
        createPointSphere(btVector3((float)(point(0) - 0.01 * direction(0)), (float)(point(1) - 0.01 * direction(1)),
                                    (float)(point(2) - 0.01 * direction(2))));
    resultCallback.clearData();
    dynamicsWorld_->contactTest(pointTemp, resultCallback);

    delete pointTemp->getCollisionShape();
    delete pointTemp->getMotionState();
    delete pointTemp;

    if (resultCallback.collisionVec.size() < oldNum)
    {
      needProj = false;
    }
    /////////////////////////////////////////////////////////////////////////

    btCollisionWorld::ClosestRayResultCallback closestResults(from, to);
    dynamicsWorld_->rayTest(from, to, closestResults);

    if (closestResults.hasHit() && needProj)
    {
      btVector3 p = closestResults.m_hitPointWorld;
      projection << p.getX(), p.getY(), p.getZ();

      // check for 0.0
      for (size_t i = 0; i < 3; ++i)
      {
        if (projection(i) < 1.0e-04)
        {
          projection(i) = 0.0;
        }
      }

      if (floatEqual(point(0), projection(0)) && floatEqual(point(1), projection(1)) &&
          floatEqual(point(2), projection(2)))
      {
        projection = point;
      }
      // cut for 0.00
      for (size_t i = 0; i < 3; ++i)
      {
        projection(i) = ((int)(floor((projection(i) + 1e-3) * 100))) / 100.0;
      }
    }
  }
  return projection;
}

double BulletPhysics::getSupportArea(const bpa::Box& new_box, const bpa::Box& old_box)
{
  return getArea(new_box, old_box, &BulletPhysics::getHorizontalArea);
}

double BulletPhysics::getContactArea(const bpa::Box& new_box, const bpa::Box& old_box)
{
  return getArea(new_box, old_box, &BulletPhysics::getVerticalArea);
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

btRigidBody* BulletPhysics::createPointSphere(btVector3 origin) const
{
  btCollisionShape* shape = new btSphereShape(0.0);
  shape->setMargin(0.0);
  btScalar mass = 0.0f;

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

double BulletPhysics::getArea(const bpa::Box& new_box, const bpa::Box& old_box,
                              double (BulletPhysics::*calculateArea)(const std::vector<btVector3>&) const)
{
  double area = 0.0;

  if (!isColliding(new_box, old_box))
  {
    addBox(new_box);

    if (dynamicsWorld_->getDispatcher()->getNumManifolds() == 1)
    {
      btPersistentManifold* contactManifold = dynamicsWorld_->getDispatcher()->getManifoldByIndexInternal(0);
      std::vector<btVector3> contactPoints;

      for (int j = 0; j < contactManifold->getNumContacts(); j++)
      {
        btManifoldPoint& pt = contactManifold->getContactPoint(j);

        if (pt.getDistance() <= 0.f)
        {
          const btVector3& ptA = pt.getPositionWorldOnA();
          contactPoints.push_back(ptA);
        }
      }
      area = (this->*calculateArea)(contactPoints);
    }

    btCollisionObject* obj = collisionBodies_.at(collisionBodies_.size() - 1);
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState())
    {
      delete body->getMotionState();
      delete body->getCollisionShape();
    }
    dynamicsWorld_->removeCollisionObject(obj);
    delete obj;
    collisionBodies_.pop_back();
  }

  return area;
}

double BulletPhysics::getHorizontalArea(const std::vector<btVector3>& points) const
{
  double area = 0.0;

  if (points.size() == 4)
  {
    if (points[0].z() == points[1].z() == points[2].z() == points[3].z())
    {
      btVector3 side_a = points[0] - points[1];
      btVector3 side_b = points[0] - points[2];
      area = side_a.cross(side_b).length();
    }
  }

  return area;
}

double BulletPhysics::getVerticalArea(const std::vector<btVector3>& points) const
{
  double area = 0.0;

  if (points.size() == 4)
  {
    std::set<std::pair<double, double>> points2d;
    for (const btVector3& point : points)
    {
      points2d.insert(std::make_pair(point.x(), point.y()));
    }

    if (points2d.size() == 2)
    {
      btVector3 side_a = points[0] - points[1];
      btVector3 side_b = points[0] - points[2];
      area = side_a.cross(side_b).length();
    }
  }

  return area;
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
}  // namespace bpa