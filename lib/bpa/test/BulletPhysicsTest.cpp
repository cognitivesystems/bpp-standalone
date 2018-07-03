#include <gtest/gtest.h>
#include "BulletPhysics.h"

namespace bpa
{
class BulletPhysicsTestFixture : public testing::Test
{
protected:
  void SetUp()
  {
    physicsEngine = PhysicsEngine::get(Engine::Bullet);
  }

  void TearDown()
  {
    delete physicsEngine;
  }

  PhysicsEngine* physicsEngine;
};

TEST_F(BulletPhysicsTestFixture, AddBoxToBulletPhysicsTest)
{
  Box box(1.0, 1.0, 1.0, 20.0, "BoxName", { "BoxLabel" });
  physicsEngine->addBox(box);

  EXPECT_TRUE(true);
}

TEST_F(BulletPhysicsTestFixture, AddBoxesToBulletPhysicsTest)
{
  Box box(1.0, 1.0, 1.0, 20.0, "BoxName", { "BoxLabel" });
  std::vector<Box> boxes{100, box};
  physicsEngine->addBoxes(boxes);

  EXPECT_TRUE(true);
}
}