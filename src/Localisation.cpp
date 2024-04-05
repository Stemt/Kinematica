#include "Localisation.hpp"

namespace Model{

  Localisation::Localisation()
  : control(0,0) , measurement(0,0)
  , beliefHistory({})
  , filter(nullptr)
  {}

  void Localisation::setFilter(std::shared_ptr<ILocalisationFilter> filter)
  {
    this->filter = filter;
  }
  
  std::shared_ptr<ILocalisationFilter> Localisation::getFilter()
  {
    return filter;
  }

  void Localisation::addControlUpdate(const Matrix &control)
  {
    if(this->control.size() == 0){
      this->control = control;
    }else{
      this->control += control;
    }
  }

  void Localisation::addMeasurementUpdate(const Matrix &measurement)
  {
    if(this->measurement.size() == 0){
      this->measurement = measurement;
    }else{
      this->measurement += measurement;
    }
  }

  void Localisation::updateBelief()
  {
    beliefHistory.push_back(filter->run(control));
    control = 0;
    measurement = 0;
  }

  const Matrix& Localisation::getBelief() const
  {
    return beliefHistory.back();
  }

  const std::vector<Matrix>& Localisation::getBeliefHistory() const
  {
    return beliefHistory;
  }

} // namespace Model
