#ifndef LEPH_CHRONO_HPP
#define LEPH_CHRONO_HPP

#include <iostream>
#include <chrono>
#include <string>
#include <map>

namespace Leph {

/**
 * Chrono
 *
 * Simple C++11 chrono wrapper for
 * test and benchmarking
 */
class Chrono 
{
    public:

        /**
         * Initialization empty
         */
        Chrono();
        
        /**
         * Reset chrono instance to empty
         */
        void clear();

        /**
         * Create and start a named point in time
         */
        void start(const std::string& name = "");

        /**
         * Create a named duration time with respect to
         * given point in time begin
         */
        void stop(const std::string& name = "", 
            const std::string& reference = "");

        /**
         * Return the given duration name in seconds
         * and milliseconds
         */
        double getDurationSec(const std::string& name) const;
        double getDurationMs(const std::string& name) const;

        /**
         * Write on given stream an overview of chrono state
         */
        void print(std::ostream& os = std::cout) const;

    private:

        /**
         * Typedef shortcuts
         */
        typedef std::chrono::duration<double> Duration;
        typedef std::map<std::string, Duration> DurationContainer;
        typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
        typedef std::map<std::string, TimePoint> TimePointContainer;

        /**
         *  Starting time points and 
         *  evaluated durations container 
         */
        TimePointContainer _timePoints;
        DurationContainer _durations;
};

}

#endif

