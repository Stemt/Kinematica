#include "Odometer.hpp"

namespace Model
{

  Odometer::Odometer()
  {}

  Odometer::Odometer(Robot* aRobot)
  : AbstractSensor(aRobot)
  {}

  Odometer::~Odometer()
  {}

  std::shared_ptr<AbstractStimulus> Odometer::getStimulus() const
  {
    Robot* robot = dynamic_cast<Robot*>(agent);
    std::shared_ptr<AbstractStimulus> poseStimulus(new PoseStimulus(robot->getPosition(), robot->getFront()));
    return poseStimulus;
  }

  std::shared_ptr<AbstractPercept> Odometer::getPerceptFor(std::shared_ptr<AbstractStimulus> anAbstractStimulus)
  {
    PoseStimulus* poseStimulus = dynamic_cast<PoseStimulus*>(anAbstractStimulus.get());
    if(!poseStimulus){
      return std::shared_ptr<AbstractPercept>(new TravelPercept(0));
    }
    BoundedVector position(poseStimulus->position);
    TravelPerceptPtr travelPercept(new TravelPercept((position-previousPosition).getMagnitude()));
    previousPosition = position;
    return travelPercept;
  }

} // Model
