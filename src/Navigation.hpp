#ifndef NAVIGATTION_HPP_
#define NAVIGATTION_HPP_

#include <vector>

#include "BoundedVector.hpp"
#include "Wall.hpp"
#include "Goal.hpp"

namespace Model{
  struct Pose{
    BoundedVector position;
    BoundedVector front;
  };
  
  class Navigation{
  public:
    Navigation();
  private:
    
  };
}

#endif //NAVIGATTION_HPP_
