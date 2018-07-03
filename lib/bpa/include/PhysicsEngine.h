#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <string>

namespace bpa
{
enum class Engine
{
  Bullet,
  Kineo
};

class PhysicsEngine
{
public:
  PhysicsEngine() = default;
  virtual ~PhysicsEngine() = default;

  static PhysicsEngine* get(const Engine& type);

private:
  static PhysicsEngine* instance_;
};
}  // namespace engine_factory

#endif  // PHYSICS_ENGINE_H