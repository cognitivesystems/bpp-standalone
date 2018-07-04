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
}