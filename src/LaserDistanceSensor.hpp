/*
 * LaserDistanceSensor.hpp
 *
 *  Created on: 15 Oct 2012
 *      Author: jkr
 */

#ifndef LASERDISTANCESENSOR_HPP_
#define LASERDISTANCESENSOR_HPP_

#include "BoundedVector.hpp"
#include "Config.hpp"

#include "AbstractSensor.hpp"

namespace Model
{
	// class DistanceStimulus

	/**
	 *
	 */
  class DistancePercept;
  typedef std::shared_ptr<DistancePercept> DistancePerceptPtr;

	class DistancePercept : public AbstractPercept
	{
		public:
			explicit DistancePercept(double angle, double distance) :
				angle(angle),
				distance(distance)
		{}
		double angle;
		double distance;
	};
	//	class DistancePercept

	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;
  
  class LaserDistanceSensor;
  typedef std::shared_ptr<LaserDistanceSensor> LaserDistanceSensorPtr;
	/**
	 *
	 */
	class LaserDistanceSensor : public AbstractSensor
	{
		public:
			/**
			 *
			 */
			LaserDistanceSensor();
			/**
			 *
			 */
			explicit LaserDistanceSensor( Robot* aRobot);
			/**
			 *
			 */
			virtual ~LaserDistanceSensor();
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractStimulus > getStimulus() const override;
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) override;
			/**
			 *
			 */
      virtual const BoundedVector& getPreviousHit() const;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override;
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override;
			//@}
		protected:
		private:
      BoundedVector previousHit;
	};
} // namespace Model
#endif /* LASERDISTANCESENSOR_HPP_ */
