#include "Compass.hpp"
#include "RobotWorld.hpp"
#include "PoseStimulus.hpp"
#include <random>

namespace Model
{

  Compass::Compass()
  {}

  Compass::Compass(Robot* aRobot, double radianStandardDeviation)
  : AbstractSensor(aRobot)
  , radianStandardDeviation(radianStandardDeviation)
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
    
    // use same generators for all instances of Compass
    static std::random_device randDev{};
    static std::mt19937 randGen{randDev()};
    static std::normal_distribution<double> dist{0.0,radianStandardDeviation};
    
    
    double deviation = dist(randGen);
    DirectionPerceptPtr directionPercept(new DirectionPercept(deviation+poseStimulus->front.getAngle(RobotWorld::NORTH)));
    return directionPercept;
  }

  double Compass::getDeviation() const
  {
    return radianStandardDeviation;
  }

} // Model
