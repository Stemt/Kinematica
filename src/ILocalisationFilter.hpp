#ifndef ILOCALISATION_FILTER_HPP_
#define ILOCALISATION_FILTER_HPP_

#include "Matrix.hpp"
#include <vector>

namespace Model{

class ILocalisationFilter{
  public:
    virtual ~ILocalisationFilter()
    {}
    virtual const Matrix& run(const Matrix& control, const Matrix& measurement) = 0;
};

} // Model

#endif // ILOCALISATION_FILTER_HPP_
