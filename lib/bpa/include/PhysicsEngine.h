#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <string>
#include "Box.h"

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

  void addBox(const bpa::Box& box, bool rotate = true)
  {
    createBox(box, rotate);
  }

  void addBoxes(const std::vector<bpa::Box>& boxes)
  {
    for (const bpa::Box& b : boxes)
    {
      addBox(b);
    }
  }

private:
  virtual void createBox(const bpa::Box& box, bool rotate) = 0;
  static PhysicsEngine* instance_;
};
}  // namespace engine_factory

#endif  // PHYSICS_ENGINE_H