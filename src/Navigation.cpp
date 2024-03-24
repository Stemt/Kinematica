#include "Navigation.hpp"

namespace Model{

  Navigation::Navigation(const std::vector<WallPtr>& walls, GoalPtr goal)
  : walls(walls)
  , goal(goal)
  {}


} // namespace Model
