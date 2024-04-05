#include "Odometer.hpp"
#include <random>

namespace Model
{

  Odometer::Odometer()
  {}

  Odometer::Odometer(Robot* aRobot, double pixelStandardDeviation)
  : AbstractSensor(aRobot)
  , previousPosition(aRobot->getPosition())
  , pixelStandardDeviation(pixelStandardDeviation)
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
    // use same generators for all instances of Compass
    static std::random_device randDev{};
    static std::mt19937 randGen{randDev()};
    static std::normal_distribution<double> dist{0.0,pixelStandardDeviation};

    BoundedVector position(poseStimulus->position);
    double deviation = dist(randGen);
    TravelPerceptPtr travelPercept(new TravelPercept(deviation+(position-previousPosition).getMagnitude()));
    previousPosition = position;
    return travelPercept;
  }

  double Odometer::getDeviation() const
  {
    return pixelStandardDeviation;
  }

} // Model
