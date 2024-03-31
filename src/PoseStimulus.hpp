#include "AbstractSensor.hpp"
#include "BoundedVector.hpp"

namespace Model{

  class PoseStimulus : public AbstractStimulus{
  public:
    PoseStimulus(BoundedVector position, BoundedVector front)
    : position(position)
    , front(front)
    {}
    PoseStimulus()
    : position()
    , front()
    {}
    BoundedVector position;
    BoundedVector front;
  };

} // Model
