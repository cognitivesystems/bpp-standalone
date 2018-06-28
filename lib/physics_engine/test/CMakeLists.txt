find_package(Bullet REQUIRED)

include_directories(include ${BULLET_INCLUDE_DIR})

set(PhysicsEngineTest_src BulletPhysicsTest.cpp)
add_executable(PhysicsEngine-test ${PhysicsEngineTest_src})
target_link_libraries(PhysicsEngine-test
  ${BULLET_DYNAMICS_LIBRARY}
  ${BULLET_COLLISION_LIBRARY}
  ${BULLET_MATH_LIBRARY}
  gtest
  gtest_main)
add_test(NAME PhysicsEngineTest
  COMMAND PhysicsEngine-test)