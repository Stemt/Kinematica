#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "ILocalisationFilter.hpp"
#include "Logger.hpp"
#include "Matrix.hpp"
#include "Queue.hpp"
#include "LaserDistanceSensor.hpp"
#include "Odometer.hpp"
#include "Compass.hpp"
#include <string>

namespace Model{
  class KalmanFilter : public ILocalisationFilter
  {
    public:
      KalmanFilter(
          Base::Queue< std::shared_ptr< AbstractPercept > >& perceptQueue,
          const Matrix& initialState
          )
        : ILocalisationFilter(perceptQueue)
        , belief(initialState)
        , beliefCovariance(Matrix::identity(initialState.getHeight()))
      {}

      const Matrix& run(const Matrix& control)
      {
        Matrix measurement(1,2,{0,0});
        while(perceptQueue.size() > 0){
          AbstractPerceptPtr percept = perceptQueue.dequeue().value();
          DistancePerceptPtr distancePercept = std::dynamic_pointer_cast<DistancePercept>(percept);
          if(distancePercept){
            Application::Logger::log("Laser: " + std::to_string(distancePercept->distance));
          }
          TravelPerceptPtr travelPercept = std::dynamic_pointer_cast<TravelPercept>(percept);
          if(travelPercept){
            Application::Logger::log("Odometer: " + std::to_string(travelPercept->distance));
            Matrix odometerUpdate(1,2, {travelPercept->distance,0});
            measurement += odometerUpdate;
          }
          DirectionPerceptPtr directionPercept = std::dynamic_pointer_cast<DirectionPercept>(percept);
          if(directionPercept){
            Application::Logger::log("Compass: " + std::to_string(directionPercept->angle));
            Matrix compassUpdate(1,2,{0,directionPercept->angle});
            measurement += compassUpdate;
          }
        }
        return update(control, measurement);
      }
      
      const Matrix& update(const Matrix& control, const Matrix& measurement)
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
