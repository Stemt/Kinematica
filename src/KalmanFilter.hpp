#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "ILocalisationFilter.hpp"
#include "Logger.hpp"
#include "Matrix.hpp"
#include <string>

namespace Model{
  class KalmanFilter : public ILocalisationFilter
  {
    public:
      KalmanFilter(
          const Matrix& initialState
          )
        : belief(initialState)
        , beliefCovariance(Matrix::identity(initialState.getHeight()))
      {}
      
      const Matrix& run(const Matrix& control, const Matrix& measurement) override
      {
        Matrix processCovariance(Matrix::identity(control.getHeight())*1.0f); //TODO finetune
        Matrix measurementCovariance(Matrix::identity(measurement.getHeight())*0.0f); //TODO finetune
        Matrix measurementNoise(1,2,{0,0}); // TODO draw from covariance
        
        Matrix stateTransition = Matrix::identity(belief.getHeight()); // identity because no passive transition
        
        // predict
        Matrix prediction = stateTransition.dot(belief) + control; // TODO add process noise draw from proces noise covariance
        Matrix predictionCovariance = stateTransition.dot(beliefCovariance).dot(stateTransition.transpose()) + processCovariance;

        // update using measurement
        Matrix measuredDelta(1,2,{measurement[0] * cos(measurement[1]), measurement[0]* sin(measurement[1])});
        Matrix measurementPrediciton = (belief + measuredDelta) + measurementNoise;

        // kalman gain
        Matrix kalmanGain = predictionCovariance.dot((predictionCovariance + measurementCovariance).inverse());
        
        // calculate new belief
        belief = prediction + kalmanGain.dot(measurementPrediciton - prediction);

        // adjust belief covariance
        beliefCovariance = (Matrix::identity(kalmanGain.getWidth()) - kalmanGain).dot(predictionCovariance);

        //Matrix output = newPosition + believedPositionWithUpdate;
        return belief;
        
      }
    private:
      Matrix belief;
      Matrix beliefCovariance;
  };
} // Model

#endif // KALMAN_FILTER_HPP
