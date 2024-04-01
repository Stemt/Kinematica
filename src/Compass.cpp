#include "Compass.hpp"
#include "RobotWorld.hpp"

namespace Model
{

  Compass::Compass()
  {}

  Compass::Compass(Robot* aRobot)
  : AbstractSensor(aRobot)
  {}

  Compass::~Compass()
  {}

  std::shared_ptr<AbstractStimulus> Compass::getStimulus() const
  {
    Robot* robot = dynamic_cast<Robot*>(agent);
    std::shared_ptr<AbstractStimulus> poseStimulus(new PoseStimulus(robot->getPosition(), robot->getFront()));
    return poseStimulus;
  }

  std::shared_ptr<AbstractPercept> Compass::getPerceptFor(std::shared_ptr<AbstractStimulus> anAbstractStimulus)
  {
    PoseStimulus* poseStimulus = dynamic_cast<PoseStimulus*>(anAbstractStimulus.get());
    if(!poseStimulus){
      return std::shared_ptr<AbstractPercept>(new DirectionPercept(0));
    }

    DirectionPerceptPtr directionPercept(new DirectionPercept(poseStimulus->front.getAngle(RobotWorld::NORTH)));
    return directionPercept;
  }

} // Model
