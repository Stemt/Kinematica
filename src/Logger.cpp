#include "Logger.hpp"

#include "ITraceFunction.hpp"
#include "Trace.hpp"

#include <iostream>

namespace Application
{
	/* static */bool Logger::disable = false;
	/**
	 *
	 */
	/*static*/void Logger::log( const std::string& aMessage)
	{
		Base::Trace::trace(aMessage);
    std::cerr << aMessage << std::endl;
	}
} //namespace Application
