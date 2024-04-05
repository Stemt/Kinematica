#ifndef PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_HPP_

#include "ILocalisationFilter.hpp"
#include "Matrix.hpp"
#include "Logger.hpp"
#include "LaserDistanceSensor.hpp"
#include "AbstractAgent.hpp"
#include "Conversions.hpp"
#include "Shape2DUtils.hpp"
#include "Wall.hpp"
#include "RobotWorld.hpp"
#include <cfloat>
#include <random>
#include <algorithm>

namespace Model{
  
  struct Particle{
    Particle()
    : state(1,2,{0,0})
    , weight(0.0f)
    {}
    Matrix state;
    double weight;
  };

  class ParticleFilter : public ILocalisationFilter{
    public:
      ParticleFilter(
        Base::Queue< std::shared_ptr< AbstractPercept > >& perceptQueue,
        size_t particleCount)
      : ILocalisationFilter(perceptQueue)
      , particles(particleCount)
      {
        for(auto& p : particles){
          static std::random_device rd;
          static std::default_random_engine generator(rd());
          std::uniform_real_distribution<double> canvasDistribution(0.0f,1024.0f);
          p.state[0] = canvasDistribution(generator);
          p.state[1] = canvasDistribution(generator);
          p.weight = 1.0f/particleCount;
        }
      }

      const Matrix& run(const Matrix& control){
        Matrix measurement(1,1,{0.0f,0.0f}); // distance and angle
        while(perceptQueue.size() > 0){
          AbstractPerceptPtr percept = perceptQueue.dequeue().value();
          DistancePerceptPtr distancePercept = std::dynamic_pointer_cast<DistancePercept>(percept);
          if(distancePercept){
            Application::Logger::log("Laser: " + std::to_string(distancePercept->distance));
            measurement[0] += distancePercept->distance;
            measurement[1] += distancePercept->angle;
          }
        }
        controlUpdate(control);
        measurementUpdate(measurement);
        resample();
        return resolveBelief();
      }

      const Matrix& resolveBelief(){
        double bestW = -FLT_MAX;
        size_t bestI = 0;
        for(size_t i = 0; i < particles.size(); ++i){
          if(particles[i].weight > bestW){
            bestW = particles[i].weight;
            bestI = i;
          }
        }
        return particles.at(bestI).state;
      }

      void controlUpdate(const Matrix& control){
        for(auto& p : particles){
          p.state += control;
        }
      }

      void measurementUpdate(const Matrix& measurement){
        for(auto& p : particles){
          BoundedVector laserOrigin(p.state[0],p.state[1]);
          BoundedVector direction(0.0f,1.0f);
          direction.rotate(measurement[1]);
          direction.normalise();
          static constexpr double LASER_RANGE = 500.0f;
          double distance = LASER_RANGE;
          // scale direction
          BoundedVector laserEnd = laserOrigin + direction * LASER_RANGE;
          BoundedVector intersection(laserEnd);

          
          auto walls = RobotWorld::getRobotWorld().getWalls();
          for( std::shared_ptr<Wall> wall : walls){

            if(!Utils::Shape2DUtils::intersect(wall->getPoint1(), wall->getPoint2(), laserOrigin.asPoint(), laserEnd.asPoint())){
              continue;
            }
            BoundedVector newIntersection = Utils::Shape2DUtils::getIntersection(wall->getPoint1(), wall->getPoint2(), laserOrigin.asPoint(), laserEnd.asPoint());
            double newDistance = (newIntersection-laserOrigin).getMagnitude();
            if(newDistance < distance){
              distance = newDistance;
              intersection = newIntersection;
            }
          }
          
          p.weight = pow(measurement[0]-distance,2);
        }
      }

      void resample(){
        std::sort(particles.begin(),particles.end(),
            [](const Particle& lhs, const Particle& rhs){ return lhs.weight < rhs.weight; });
        
        std::vector<Particle> resampled;
        std::vector<double> weights;
        for(auto& p : particles){
          weights.push_back(p.weight);
        }
      
        static std::random_device rd;
        static std::default_random_engine generator(rd());
        std::discrete_distribution<> resampleDistribution(weights.begin(),weights.end()); 
        std::uniform_real_distribution<double> displacementDistribution(-1.0f, 1.0f);

        for(size_t i = 0; i < particles.size(); ++i){
          Particle& p = particles.at(static_cast<size_t>(resampleDistribution(generator)));

          const double maxRange = 15.0f;
          double x = p.state[0] + (maxRange * displacementDistribution(generator));
          double y = p.state[1] + (maxRange * displacementDistribution(generator));

          x = (x < 0) ? 0 : x;
          x = (x > 1024.0f) ? 1024.0f : x;
          y = (y < 0) ? 0 : y;
          y = (y > 1024.0f) ? 1024.0f : y;

          p.state[0] = x;
          p.state[1] = y;
          resampled.push_back(p);
        }

        particles = resampled;
      }

    private:
      std::vector<Particle> particles;

  };

}

#endif // PARTICLE_FILTER_HPP_
