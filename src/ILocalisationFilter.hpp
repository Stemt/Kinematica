#ifndef ILOCALISATION_FILTER_HPP_
#define ILOCALISATION_FILTER_HPP_

#include "Matrix.hpp"
#include "AbstractSensor.hpp"
#include "Queue.hpp"

namespace Model{

class ILocalisationFilter{
  public:
    ILocalisationFilter(
        Base::Queue< std::shared_ptr< AbstractPercept > >& perceptQueue)
    : perceptQueue(perceptQueue)
    {}

    virtual ~ILocalisationFilter()
    {}
    virtual const Matrix& run(const Matrix& control) = 0;
  protected:
    Base::Queue< std::shared_ptr< AbstractPercept > >& perceptQueue;
};

} // Model

#endif // ILOCALISATION_FILTER_HPP_
