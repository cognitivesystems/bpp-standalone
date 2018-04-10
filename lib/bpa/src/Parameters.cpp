#include "Parameters.h"

namespace bpa
{
Params* Params::singleton_ = nullptr;

Params* Params::instance()
{
  if (singleton_ == nullptr)
  {
    singleton_ = new Params;
  }
  return singleton_;
}
}
