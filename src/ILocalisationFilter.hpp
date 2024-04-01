#ifndef ILOCALISATION_FILTER_HPP_
#define ILOCALISATION_FILTER_HPP_

#include <vector>

class ILocalisationFilter{
  public:
    virtual ~ILocalisationFilter();
    virtual void run(const std::vector<double>& measurement);
};

#endif // ILOCALISATION_FILTER_HPP_
