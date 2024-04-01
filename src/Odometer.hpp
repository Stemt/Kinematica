#ifndef ODOMETER_HPP
#define ODOMETER_HPP

#include "AbstractSensor.hpp"
#include "Robot.hpp"
#include "PoseStimulus.hpp"

namespace Model{
  class TravelPercept;
  typedef std::shared_ptr<TravelPercept> TravelPerceptPtr;
  
  class TravelPercept : public AbstractPercept
  {
    public:
      explicit TravelPercept(double distance)
      : distance(distance)
      {}
      double distance;
  };

  class Odometer : public AbstractSensor
  {
    public:
      Odometer();
      explicit Odometer(Robot* robot);
      virtual ~Odometer();
      virtual std::shared_ptr<AbstractStimulus> getStimulus() const override;
      virtual std::shared_ptr<AbstractPercept> getPerceptFor(std::shared_ptr<AbstractStimulus> anAbstractStimulus) override;
    protected:
      BoundedVector previousPosition;
  };
} // Model

#endif // ODOMETER_HPP
