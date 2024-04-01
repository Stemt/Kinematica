#ifndef COMPASS_HPP
#define COMPASS_HPP

#include "AbstractSensor.hpp"
#include "Robot.hpp"
#include "PoseStimulus.hpp"

namespace Model{
  class DirectionPercept;
  typedef std::shared_ptr<DirectionPercept> DirectionPerceptPtr;
  
  class DirectionPercept : public AbstractPercept
  {
    public:
      explicit DirectionPercept(double angle)
      : angle(angle)
      {}
      double angle;
  };

  class Compass : public AbstractSensor
  {
    public:
      Compass();
      explicit Compass(Robot* robot);
      virtual ~Compass();
      virtual std::shared_ptr<AbstractStimulus> getStimulus() const override;
      virtual std::shared_ptr<AbstractPercept> getPerceptFor(std::shared_ptr<AbstractStimulus> anAbstractStimulus) override;
  };
} // Model

#endif // COMPASS_HPP
