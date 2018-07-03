#include "BulletPhysics.h"
#include "KineoPhysics.h"

namespace bpa
{
PhysicsEngine* PhysicsEngine::instance_ = nullptr;

PhysicsEngine* PhysicsEngine::get(const Engine& type)
{
  if (!instance_)
  {
    switch (type)
    {
      case Engine::Bullet:
        return new BulletPhysics();
      case Engine::Kineo:
        return new KineoPhysics();
    }
  }
  return instance_;
}
}