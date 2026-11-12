#pragma once
#include <cmath>

/** Runtime search pitch only. Positive values tilt DOWN; never a body permit. */
namespace WhiteFleetK1SearchHead
{
  inline bool valid(float value) noexcept {return std::isfinite(value) && value >= 0.f && value <= 40.f;}
  inline bool valid(float search, float unknown) noexcept {return valid(search) && valid(unknown);}
}
