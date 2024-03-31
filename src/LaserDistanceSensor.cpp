/*
 * LaserDistanceSensor.cpp
 *
 *  Created on: 15 Oct 2012
 *      Author: jkr
 */

#include "LaserDistanceSensor.hpp"

#include "BoundedVector.hpp"
//#include "Geometry.hpp"
#include "PoseStimulus.hpp"
#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Shape2DUtils.hpp"

namespace Model
{

	/**
	 *
	 */
	LaserDistanceSensor::LaserDistanceSensor()
	{
	}
	/**
	 *
	 */
	LaserDistanceSensor::LaserDistanceSensor( Robot* aRobot) :
								AbstractSensor( aRobot)
	{
	}
	/**
	 *
	 */
	LaserDistanceSensor::~LaserDistanceSensor()
	{
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractStimulus > LaserDistanceSensor::getStimulus() const
	{
    Robot* robot = dynamic_cast<Robot*>(agent);
		std::shared_ptr< AbstractStimulus > distanceStimulus( new PoseStimulus(robot->getPosition(),robot->getFront()));
		return distanceStimulus;
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractPercept > LaserDistanceSensor::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus)
	{
		PoseStimulus* poseStimulus = dynamic_cast< PoseStimulus* >( anAbstractStimulus.get());
    BoundedVector laserOrigin = poseStimulus->position;
    BoundedVector direction = poseStimulus->front;
    direction.normalise();
    static constexpr double LASER_RANGE = 500.0f;
    double distance = LASER_RANGE;
    // scale direction
    BoundedVector laserEnd = laserOrigin + direction * LASER_RANGE;
    BoundedVector intersection(laserEnd);

    
    auto walls = RobotWorld::getRobotWorld().getWalls();
    Application::Logger::log("Check N wall: "+std::to_string(walls.size()));
    for( std::shared_ptr<Wall> wall : walls){

      Application::Logger::log("  > Laser{: " + laserOrigin.asString() + ", " + laserEnd.asString()+"}");
      Application::Logger::log("  > Wall{(" + std::to_string(wall->getPoint1().x) + ", "+ std::to_string(wall->getPoint1().y) + "), (" + std::to_string(wall->getPoint2().x) + ", " + std::to_string(wall->getPoint2().y) +")" );
      if(!Utils::Shape2DUtils::intersect(wall->getPoint1(), wall->getPoint2(), laserOrigin.asPoint(), laserEnd.asPoint())){
        continue;
      }
      BoundedVector newIntersection = Utils::Shape2DUtils::getIntersection(wall->getPoint1(), wall->getPoint2(), laserOrigin.asPoint(), laserEnd.asPoint());
      Application::Logger::log("  >> HIT at " + newIntersection.asString());
      double newDistance = (newIntersection-laserOrigin).getMagnitude();
      if(newDistance < distance){
        distance = newDistance;
        intersection = newIntersection;
        Application::Logger::log("  >>> new Dist! "+std::to_string(distance));
      }else{
        Application::Logger::log("  >> Discarded: " +std::to_string(newDistance)+ " > " +std::to_string(distance) );
      }
    }
    previousHit = intersection;
    double angle = (intersection-laserOrigin).getAngle(RobotWorld::NORTH);


		return std::shared_ptr< AbstractPercept >( new DistancePercept( angle, distance));
	}

  const BoundedVector& LaserDistanceSensor::getPreviousHit() const
  {
    return previousHit;
  }
	/**
	 *
	 */
	std::string LaserDistanceSensor::asString() const
	{
		return "LaserDistanceSensor";
	}
	/**
	 *
	 */
	std::string LaserDistanceSensor::asDebugString() const
	{
		return asString();
	}
} // namespace Model
