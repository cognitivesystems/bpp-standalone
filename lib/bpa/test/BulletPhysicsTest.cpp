#include <gtest/gtest.h>
#include "BulletPhysics.h"

namespace bpa
{
class BulletPhysicsTestFixture : public testing::Test
{
protected:
  void SetUp()
  {
    bulletPhysics = BulletPhysics::instance();
  }

  void TearDown()
  {
    delete bulletPhysics;
  }

  BulletPhysics* bulletPhysics;
};

/**
 * used with valgrind memcheck to detect memory leaks
 */
TEST_F(BulletPhysicsTestFixture, AddBoxToBulletPhysicsTest)
{
  Box box(1.0, 1.0, 1.0, 20.0, "BoxName", { "BoxLabel" });
  bulletPhysics->addBox(box);

  EXPECT_TRUE(true);
}

/**
 * used with valgrind memcheck to detect memory leaks
 */
TEST_F(BulletPhysicsTestFixture, AddBoxesToBulletPhysicsTest)
{
  Box box(1.0, 1.0, 1.0, 20.0, "BoxName", { "BoxLabel" });
  std::vector<Box> boxes{ 100, box };
  bulletPhysics->addBoxes(boxes);

  EXPECT_TRUE(true);
}

TEST_F(BulletPhysicsTestFixture, OneBoxCollisionTest)
{
  Box box(1.0, 1.0, 1.0, 20.0, "BoxName", { "BoxLabel" });
  EXPECT_FALSE(bulletPhysics->isColliding(box));

  bulletPhysics->addBox(box);
  EXPECT_EQ(1, bulletPhysics->numCollisionObjects());
  EXPECT_TRUE(bulletPhysics->isColliding(box));

  box.position.position(0) = 0.5;
  box.position.position(1) = 0.5;
  EXPECT_TRUE(bulletPhysics->isColliding(box));

  // we are still able to detect this collision
  box.position.position(1) = 0.99999;
  EXPECT_TRUE(bulletPhysics->isColliding(box));

  // but not this
  box.position.position(1) = 0.999999;
  EXPECT_FALSE(bulletPhysics->isColliding(box));

  // the precision is controlled by FLOAT_EPS defined in Parameters.h
  // it cannot be more precise than current value, otherwise the following will be reported as collision
  box.position.position(1) = 1.0;
  EXPECT_FALSE(bulletPhysics->isColliding(box));

  EXPECT_EQ(1, bulletPhysics->numCollisionObjects());
}

TEST_F(BulletPhysicsTestFixture, TwoBoxesCollisionTest)
{
  Box box_a(1.0, 1.0, 1.0, 20.0, "BoxName", { "BoxLabel" });
  Box box_b(1.0, 1.0, 1.0, 20.0, "BoxName", { "BoxLabel" });
  EXPECT_TRUE(bulletPhysics->isColliding(box_a, box_b));

  box_a.position.position(0) = 0.5;
  box_a.position.position(1) = 0.5;
  EXPECT_TRUE(bulletPhysics->isColliding(box_a, box_b));

  // we are only able to detect this collision
  box_a.position.position(1) = 0.99;
  EXPECT_TRUE(bulletPhysics->isColliding(box_a, box_b));

  // but not this
  box_a.position.position(1) = 0.999;
  EXPECT_FALSE(bulletPhysics->isColliding(box_a, box_b));

  box_a.position.position(1) = 1.0;
  EXPECT_FALSE(bulletPhysics->isColliding(box_a, box_b));

  EXPECT_EQ(0, bulletPhysics->numCollisionObjects());
}

TEST_F(BulletPhysicsTestFixture, PointContactTest)
{
  Eigen::Vector3d point_a(0.5, 0.5, 0.5);
  EXPECT_FALSE(bulletPhysics->isPointContact(point_a));

  Box box(1.0, 1.0, 1.0, 20.0, "BoxName", { "BoxLabel" });
  bulletPhysics->addBox(box);
  EXPECT_TRUE(bulletPhysics->isPointContact(point_a));

  // we are only able to detect this collision
  Eigen::Vector3d point_b(0.9999, 0.9999, 0.9999);
  EXPECT_TRUE(bulletPhysics->isPointContact(point_b));

  // but not this
  Eigen::Vector3d point_c(0.99999, 0.99999, 0.99999);
  EXPECT_FALSE(bulletPhysics->isPointContact(point_c));

  Eigen::Vector3d point_d(1.0, 1.0, 1.0);
  EXPECT_FALSE(bulletPhysics->isPointContact(point_d));

  EXPECT_EQ(1, bulletPhysics->numCollisionObjects());
}
}