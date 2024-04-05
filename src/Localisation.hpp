#ifndef NAVIGATTION_HPP_
#define NAVIGATTION_HPP_

#include <vector>

#include "BoundedVector.hpp"
#include "Wall.hpp"
#include "Goal.hpp"
#include "ILocalisationFilter.hpp"

namespace Model{
  
  class Localisation{
  public:
    Localisation();
    void setFilter(std::shared_ptr<ILocalisationFilter> filter);
    std::shared_ptr<ILocalisationFilter> getFilter();
    void updateBelief();
    void addControlUpdate(const Matrix& control);
    void addMeasurementUpdate(const Matrix& measurement);
    const Matrix& getBelief() const;
    const std::vector<Matrix>& getBeliefHistory() const;

    Matrix control;
    Matrix measurement;
    std::vector<Matrix> beliefHistory;

    std::shared_ptr<ILocalisationFilter> filter;
  };
}

#endif //NAVIGATTION_HPP_
