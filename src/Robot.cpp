#include "Robot.hpp"

#include "BoundedVector.hpp"
#include "Client.hpp"
#include "CommunicationService.hpp"
#include "Conversions.hpp"
#include "Goal.hpp"
#include "ILocalisationFilter.hpp"
#include "KalmanFilter.hpp"
#include "LaserDistanceSensor.hpp"
#include "Odometer.hpp"
#include "Compass.hpp"
#include "Logger.hpp"
#include "MainApplication.hpp"
#include "MathUtils.hpp"
#include "Message.hpp"
#include "MessageTypes.hpp"
#include "ParticleFilter.hpp"
#include "RobotWorld.hpp"
#include "Server.hpp"
#include "Shape2DUtils.hpp"
#include "Wall.hpp"
#include "WayPoint.hpp"

#include <chrono>
#include <ctime>
#include <memory>
#include <sstream>
#include <thread>

namespace Model
{
	/**
	 *
	 */
	Robot::Robot() :
								name( ""),
								size( DefaultSize),
								position( DefaultPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false)
	{
    setupSensors();
	}
	/**
	 *
	 */
	Robot::Robot( const std::string& aName) :
								name( aName),
								size( DefaultSize),
								position( DefaultPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false)
	{
    setupSensors();
	}
	/**
	 *
	 */
	Robot::Robot(	const std::string& aName,
					const Point& aPosition) :
								name( aName),
								size( DefaultSize),
								position( aPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false)
	{
    setupSensors();
	}
	/**
	 *
	 */
	Robot::~Robot()
	{
		if(driving)
		{
			stopDriving();
		}
		if(acting)
		{
			stopActing();
		}
		if(communicating)
		{
			stopCommunicating();
		}
	}
	/**
	 *
	 */
  void Robot::setupSensors()
  {
		std::shared_ptr< AbstractSensor > laserSensor( new LaserDistanceSensor( this));
		attachSensor( laserSensor);
    std::shared_ptr<AbstractSensor> odometer(new Odometer(this,10.0f));
    attachSensor(odometer);
    std::shared_ptr<AbstractSensor> compass(new Compass(this,Utils::deg2grad(2.0f)));
    attachSensor(compass);
    switchFilter(); // initialize filter
  }
	/**
	 *
	 */
	void Robot::setName( const std::string& aName,
						 bool aNotifyObservers /*= true*/)
	{
		name = aName;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}

	}
	/**
	 *
	 */
	Size Robot::getSize() const
	{
		return size;
	}
	/**
	 *
	 */
	void Robot::setSize(	const Size& aSize,
							bool aNotifyObservers /*= true*/)
	{
		size = aSize;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::setPosition(	const Point& aPosition,
								bool aNotifyObservers /*= true*/)
	{
		position = aPosition;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */ BoundedVector Robot::getFront() const
	{
		return front;
	}
	/**
	 *
	 */
	void Robot::setFront(	const BoundedVector& aVector,
							bool aNotifyObservers /*= true*/)
	{
		front = aVector;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	float Robot::getSpeed() const
	{
		return speed;
	}
	/**
	 *
	 */
	void Robot::setSpeed( float aNewSpeed,
						  bool aNotifyObservers /*= true*/)
	{
		speed = aNewSpeed;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::startActing()
	{
		acting = true;
		std::thread newRobotThread( [this]{	startDriving();});
		robotThread.swap( newRobotThread);
	}
	/**
	 *
	 */
	void Robot::stopActing()
	{
		acting = false;
		driving = false;
		robotThread.join();
	}
	/**
	 *
	 */
	void Robot::startDriving()
	{
		driving = true;

		goal = RobotWorld::getRobotWorld().getGoal( "Goal");
		calculateRoute(goal);

		drive();
	}
	/**
	 *
	 */
	void Robot::stopDriving()
	{
		driving = false;
	}
	/**
	 *
	 */
	void Robot::startCommunicating()
	{
		if(!communicating)
		{
			communicating = true;

			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven( "-local_port"))
			{
				localPort = Application::MainApplication::getArg( "-local_port").value;
			}

			if(Messaging::CommunicationService::getCommunicationService().isStopped())
			{
				TRACE_DEVELOP( "Restarting the Communication service");
				Messaging::CommunicationService::getCommunicationService().restart();
			}

			server = std::make_shared<Messaging::Server>(	static_cast<unsigned short>(std::stoi(localPort)),
															toPtr<Robot>());
			Messaging::CommunicationService::getCommunicationService().registerServer( server);
		}
	}
	/**
	 *
	 */
	void Robot::stopCommunicating()
	{
		if(communicating)
		{
			communicating = false;

			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven( "-local_port"))
			{
				localPort = Application::MainApplication::getArg( "-local_port").value;
			}

			Messaging::Client c1ient( 	"localhost",
										static_cast<unsigned short>(std::stoi(localPort)),
										toPtr<Robot>());
			Messaging::Message message( Messaging::StopCommunicatingRequest, "stop");
			c1ient.dispatchMessage( message);
		}
	}
	/**
	 *
	 */
  void Robot::switchFilter(){
    if(driving){
      Application::Logger::log("Robot must be stopped to switch filters");
    };
    std::shared_ptr<ILocalisationFilter> filter = localisation.getFilter();
    if(std::dynamic_pointer_cast<ParticleFilter>(filter)){
      Matrix belief(1,2,{position.x,position.y});
      localisation.setFilter(std::shared_ptr<ILocalisationFilter>(new KalmanFilter(perceptQueue,belief)));
      Application::Logger::log("Switched to Kalman filter!");
    }else{
      localisation.setFilter(std::shared_ptr<ILocalisationFilter>(new ParticleFilter(perceptQueue,10000)));
      Application::Logger::log("Switched to Particle filter!");
    }
  }
	/**
	 *
	 */
	Region Robot::getRegion() const
	{
		Point translatedPoints[] = { getFrontRight(), getFrontLeft(), getBackLeft(), getBackRight() };
		return Region( 4, translatedPoints);
	}
	/**
	 *
	 */
  const std::vector<AbstractSensorPtr>& Robot::getSensors()
  {
    return sensors;
  }
	/**
	 *
	 */
	bool Robot::intersects( const Region& aRegion) const
	{
		Region region = getRegion();
		region.Intersect( aRegion);
		return !region.IsEmpty();
	}
	/**
	 *
	 */
	Point Robot::getFrontLeft() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalFrontLeft( x, y);
		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point frontLeft( static_cast<int>((originalFrontLeft.x - position.x) * std::cos( angle) - (originalFrontLeft.y - position.y) * std::sin( angle) + position.x),
						 static_cast<int>((originalFrontLeft.y - position.y) * std::cos( angle) + (originalFrontLeft.x - position.x) * std::sin( angle) + position.y));

		return frontLeft;
	}
	/**
	 *
	 */
	Point Robot::getFrontRight() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalFrontRight( x + size.x, y);
		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point frontRight( static_cast<int>((originalFrontRight.x - position.x) * std::cos( angle) - (originalFrontRight.y - position.y) * std::sin( angle) + position.x),
						  static_cast<int>((originalFrontRight.y - position.y) * std::cos( angle) + (originalFrontRight.x - position.x) * std::sin( angle) + position.y));

		return frontRight;
	}
	/**
	 *
	 */
	Point Robot::getBackLeft() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalBackLeft( x, y + size.y);

		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point backLeft( static_cast<int>((originalBackLeft.x - position.x) * std::cos( angle) - (originalBackLeft.y - position.y) * std::sin( angle) + position.x),
						static_cast<int>((originalBackLeft.y - position.y) * std::cos( angle) + (originalBackLeft.x - position.x) * std::sin( angle) + position.y));

		return backLeft;

	}
	/**
	 *
	 */
	Point Robot::getBackRight() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalBackRight( x + size.x, y + size.y);

		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point backRight( static_cast<int>((originalBackRight.x - position.x) * std::cos( angle) - (originalBackRight.y - position.y) * std::sin( angle) + position.x),
						 static_cast<int>((originalBackRight.y - position.y) * std::cos( angle) + (originalBackRight.x - position.x) * std::sin( angle) + position.y));

		return backRight;
	}
	/**
	 *
	 */
	void Robot::handleNotification()
	{
		//	std::unique_lock<std::recursive_mutex> lock(robotMutex);

		static int update = 0;
		if ((++update % 200) == 0)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::handleRequest( Messaging::Message& aMessage)
	{
		FUNCTRACE_TEXT_DEVELOP(aMessage.asString());

		switch(aMessage.getMessageType())
		{
			case Messaging::StopCommunicatingRequest:
			{
				aMessage.setMessageType(Messaging::StopCommunicatingResponse);
				aMessage.setBody("StopCommunicatingResponse");
				// Handle the request. In the limited context of this works. I am not sure
				// whether this works OK in a real application because the handling is time sensitive,
				// i.e. 2 async timers are involved:
				// see CommunicationService::stopServer and Server::stopHandlingRequests
				Messaging::CommunicationService::getCommunicationService().stopServer(12345,true);

				break;
			}
			case Messaging::EchoRequest:
			{
				aMessage.setMessageType(Messaging::EchoResponse);
				aMessage.setBody( "Messaging::EchoResponse: " + aMessage.asString());
				break;
			}
			default:
			{
				TRACE_DEVELOP(__PRETTY_FUNCTION__ + std::string(": default not implemented"));
				break;
			}
		}
	}
	/**
	 *
	 */
	void Robot::handleResponse( const Messaging::Message& aMessage)
	{
		FUNCTRACE_TEXT_DEVELOP(aMessage.asString());

		switch(aMessage.getMessageType())
		{
			case Messaging::StopCommunicatingResponse:
			{
				//Messaging::CommunicationService::getCommunicationService().stop();
				break;
			}
			case Messaging::EchoResponse:
			{
				break;
			}
			default:
			{
				TRACE_DEVELOP(__PRETTY_FUNCTION__ + std::string( ": default not implemented, ") + aMessage.asString());
				break;
			}
		}
	}
	/**
	 *
	 */
	std::string Robot::asString() const
	{
		std::ostringstream os;

		os << "Robot " << name << " at (" << position.x << "," << position.y << ")";

		return os.str();
	}
	/**
	 *
	 */
	std::string Robot::asDebugString() const
	{
		std::ostringstream os;

		os << "Robot:\n";
		os << AbstractAgent::asDebugString();
		os << "Robot " << name << " at (" << position.x << "," << position.y << ")\n";

		return os.str();
	}
	/**
	 *
	 */
	void Robot::drive()
	{
    double previousHeading = getHeading();
    Application::Logger::log(__PRETTY_FUNCTION__);
		try
		{

      
			for (std::shared_ptr< AbstractSensor > sensor : sensors)
			{
				sensor->setOn();
			}

			if (speed == 0.0)
			{
				speed = 10.0;
			}

			unsigned pathPoint = 0;
			while (pathPoint < path.size())
			{
        
        BoundedVector heading = front * speed;
        Matrix controlUpdate(1,2,{heading.x,heading.y});
        localisation.addControlUpdate(controlUpdate);
        localisation.updateBelief();

				const PathAlgorithm::Vertex& vertex = path[pathPoint+=static_cast<int>(speed)];
				Application::Logger::log("new speed: " + std::to_string(speed));
				front = BoundedVector( vertex.asPoint(), position);
				position.x = vertex.x;
				position.y = vertex.y;

				if (arrived(goal) || collision())
				{
					Application::Logger::log(__PRETTY_FUNCTION__ + std::string(": arrived or collision"));
					notifyObservers();
					break;
				}

				notifyObservers();

				std::this_thread::sleep_for( std::chrono::milliseconds( 100));

				// this should be the last thing in the loop
				if(driving == false)
				{
					return;
				}
			} // while

			for (std::shared_ptr< AbstractSensor > sensor : sensors)
			{
				sensor->setOff();
			}
		}
		catch (std::exception& e)
		{
			Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": ") + e.what());
			std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
		}
		catch (...)
		{
			Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": unknown exception"));
			std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
		}
	}
	/**
	 *
	 */
	void Robot::calculateRoute(GoalPtr aGoal)
	{
		path.clear();
		if (aGoal)
		{
			// Turn off logging if not debugging AStar
			Application::Logger::setDisable();

			front = BoundedVector( aGoal->getPosition(), position);
			handleNotificationsFor( astar);
			path = astar.search( position, aGoal->getPosition(), size);
			stopHandlingNotificationsFor( astar);

			Application::Logger::setDisable( false);
		}
	}
	/**
	 *
	 */
	bool Robot::arrived(GoalPtr aGoal)
	{
		if (aGoal && intersects( aGoal->getRegion()))
		{
			return true;
		}
		return false;
	}
	/**
	 *
	 */
	bool Robot::collision()
	{
		Point frontLeft = getFrontLeft();
		Point frontRight = getFrontRight();
		Point backLeft = getBackLeft();
		Point backRight = getBackRight();

		const std::vector< WallPtr >& walls = RobotWorld::getRobotWorld().getWalls();
		for (WallPtr wall : walls)
		{
			if (Utils::Shape2DUtils::intersect( frontLeft, frontRight, wall->getPoint1(), wall->getPoint2()) ||
							Utils::Shape2DUtils::intersect( frontLeft, backLeft, wall->getPoint1(), wall->getPoint2())	||
							Utils::Shape2DUtils::intersect( frontRight, backRight, wall->getPoint1(), wall->getPoint2()))
			{
				return true;
			}
		}
		const std::vector< RobotPtr >& robots = RobotWorld::getRobotWorld().getRobots();
		for (RobotPtr robot : robots)
		{
			if ( getObjectId() == robot->getObjectId())
			{
				continue;
			}
			if(intersects(robot->getRegion()))
			{
				return true;
			}
		}
		return false;
	}

} // namespace Model
