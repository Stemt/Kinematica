#ifndef COMPASS_HPP
#define COMPASS_HPP

#include "AbstractSensor.hpp"
#include "Robot.hpp"

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
      explicit Compass(Robot* robot, double radianStandardDeviation);
      virtual ~Compass();
      virtual std::shared_ptr<AbstractStimulus> getStimulus() const override;
      virtual std::shared_ptr<AbstractPercept> getPerceptFor(std::shared_ptr<AbstractStimulus> anAbstractStimulus) override;
      virtual double getDeviation() const;
    private:
      double radianStandardDeviation;
  };
} // Model

#endif // COMPASS_HPP
