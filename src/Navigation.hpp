#ifndef NAVIGATTION_HPP_
#define NAVIGATTION_HPP_

#include <vector>

#include "Wall.hpp"
#include "Goal.hpp"

namespace Model{

  class Navigation{
  public:
    Navigation(const std::vector<WallPtr>& walls, GoalPtr goal);
  private:
    std::vector<WallPtr> walls;
    GoalPtr goal;
  };
}

#endif //NAVIGATTION_HPP_
