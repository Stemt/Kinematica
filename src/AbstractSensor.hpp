#ifndef ABSTRACTSENSOR_HPP_
#define ABSTRACTSENSOR_HPP_

#include "Config.hpp"

#include "ModelObject.hpp"

#include <mutex>
#include <thread>


namespace Model
{
	class AbstractAgent;
	typedef std::shared_ptr< AbstractAgent > AbstractAgentPtr;

	/**
	 *
	 */
	class AbstractStimulus
	{
		public:
			/**
			 *
			 */
			virtual ~AbstractStimulus()
			{
			}
	};
	// class AbstractStimulus
	/**
	 *
	 */
	class AbstractPercept
	{
		public:
			/**
			 *
			 */
			virtual ~AbstractPercept()
			{
			}
	};
	// class AbstractPercept

	class AbstractSensor : public ModelObject
	{
		public:
			/**
			 *
			 */
			AbstractSensor();
			/**
			 *
			 */
			explicit AbstractSensor( AbstractAgent* anAgent);
			/**
			 *
			 */
			virtual ~AbstractSensor();
			/**
			 * A sensor reads 10 stimuli/second (it sleeps for 100 ms) by default
			 */
			virtual void setOn( unsigned long aSleepTime = 100);
			/**
			 *
			 */
			virtual void setOff();
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractStimulus > getStimulus() const = 0;
			/**
			 *
			 */
      virtual std::shared_ptr<AbstractPercept> getPreviousPercept() const;
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractPercepts) = 0;
			/**
			 *
			 */
			virtual void sendPercept( std::shared_ptr< AbstractPercept > anAbstractPercept);
			/**
			 *
			 */
			virtual void run( unsigned long aSleepTime);
			/**
			 *
			 */
			virtual void attachAgent( AbstractAgent* anAgent);
			/**
			 *
			 */
			virtual void detachAgent();
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
			AbstractAgent* agent;
			bool running;
			std::thread sensorThread;
			mutable std::recursive_mutex sensorMutex;
      std::shared_ptr<AbstractPercept> previousPercept;
		private:
	};
// class AbstractSensor
}// namespace Model

#endif // ABSTRACTSENSOR_HPP_
