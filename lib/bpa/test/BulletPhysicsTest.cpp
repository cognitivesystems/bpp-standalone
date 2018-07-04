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
  EXPECT_TRUE(bulletPhysics->isColliding(box));

  box.position.position(0) = 0.5;
  box.position.position(1) = 0.5;
  EXPECT_TRUE(bulletPhysics->isColliding(box));

  // bullet is able to detect this collision
  box.position.position(1) = 0.999;
  EXPECT_TRUE(bulletPhysics->isColliding(box));

  // but not this
  box.position.position(1) = 0.9999;
  EXPECT_FALSE(bulletPhysics->isColliding(box));

  box.position.position(1) = 1.0;
  EXPECT_FALSE(bulletPhysics->isColliding(box));
}
}