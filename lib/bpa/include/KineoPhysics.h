#ifndef KINEO_PHYSICS_H
#define KINEO_PHYSICS_H

#include "PhysicsEngine.h"

namespace bpa
{
class KineoPhysics : public PhysicsEngine
{
public:
  KineoPhysics() = default;
  virtual ~KineoPhysics() = default;

private:
  virtual void createBox(const bpa::Box& box, bool rotate);
};
}  // namespace engine_factory

#endif  // KINEO_PHYSICS_H