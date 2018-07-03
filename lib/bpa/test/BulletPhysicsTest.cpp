#include <gtest/gtest.h>

namespace physics_engine
{
class BulletPhysicsTestFixture : public testing::Test
{
protected:
  void SetUp()
  {
  }

  void TearDown()
  {
  }
};

TEST_F(BulletPhysicsTestFixture, DummyTest)
{
  EXPECT_TRUE(true);
}
}