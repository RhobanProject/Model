#ifndef LEPH_TIMESERIES_HPP
#define LEPH_TIMESERIES_HPP

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <stdexcept>
#include <limits>
#include <cmath>

namespace Leph {

/**
 * TimeSeries
 *
 * Simple time series 
 * contained associated with a name.
 * Internal container is implemented
 * as a rolling buffer to save memory.
 */
class TimeSeries
{
    public:

        /**
         * Structure for timed value
         * time is in second
         */
        struct Point {
            double time;
            double value;
        };

        /**
         * Initialization with empty or
         * given name
         * maxSize is maximum size of internal rolling buffer.
         * Infinite size is assumed if maxSize is -1
         */
        TimeSeries() :
            _name(""),
            _maxSize(-1),
            _indexEnd(0),
            _data(),
            _count(0),
            _min(std::numeric_limits<double>::quiet_NaN()),
            _max(std::numeric_limits<double>::quiet_NaN())
        {
        }
        TimeSeries(
            const std::string& name, 
            size_t maxSize = -1) :
            _name(name),
            _maxSize(maxSize),
            _indexEnd(0),
            _data(),
            _count(0),
            _min(std::numeric_limits<double>::quiet_NaN()),
            _max(std::numeric_limits<double>::quiet_NaN())
        {
        }

        /**
         * Return TimeSeries name
         */
        inline const std::string& name() const
        {
            return _name;
        }

        /**
         * Return the number of currently
         * stored data points
         */
        inline size_t size() const
        {
            if (_maxSize == (size_t)-1 || _data.size() < _maxSize) {
                return _data.size();
            } else {
                return _maxSize;
            }
        }

        /**
         * Reset the series data to empty
         */
        inline void clear()
        {
            _indexEnd = 0;
            _data.clear();
            _count = 0;
            _min = std::numeric_limits<double>::quiet_NaN();
            _max = std::numeric_limits<double>::quiet_NaN();
        }

        /**
         * Return newest and oldest 
         * inserted point time
         */
        inline double timeMin() const
        {
            return at(size()-1).time;
        }
        inline double timeMax() const
        {
            return at(0).time;
        }

        /**
         * Return the total number of
         * added data points
         */
        inline size_t count() const
        {
            return _count;
        }

        /**
         * Return minimum and maximum
         * value ever added
         * NaN is returned if no
         * points was added
         */
        inline double min() const
        {
            if (std::isnan(_min)) {
                throw std::logic_error("TimeSeries empty");
            }
            return _min;
        }
        inline double max() const
        {
            if (std::isnan(_max)) {
                throw std::logic_error("TimeSeries empty");
            }
            return _max;
        }
        
        /**
         * Compute and do linear interpolation
         * of series values at given time.
         * NaN is returned if out of bound
         * time is asked.
         */
        inline double get(double time) const
        {
            if (size() < 2) {
                throw std::logic_error("TimeSeries not enough points");
            }
            if (time < timeMin() || time > timeMax()) {
                throw std::logic_error("TimeSeries out of bound");
            }

            //Bijection search
            size_t indexLow = 0;
            size_t indexUp = size()-1;
            while (indexUp-indexLow != 1) {
                size_t i = (indexLow+indexUp)/2;
                if (at(i).time <= time) {
                    indexUp = i;
                } else {
                    indexLow = i;
                }
            }

            //Linear interpolation
            double d1 = at(indexLow).time - at(indexUp).time;
            double d2 = time - at(indexUp).time;
            return at(indexUp).value*(d1-d2)/d1 + at(indexLow).value*d2/d1;
        }

        /**
         * Direct access to Points container
         * from index 0 to size()-1.
         * Indexes goes from newest inserted point
         * to oldest inserted point
         */
        inline const Point& operator[](size_t index) const
        {
            if (_maxSize == (size_t)-1 || _data.size() < _maxSize) {
                return _data.at(_data.size()-1-index);
            } else {
                int i = _indexEnd-1-index;
                if (i < 0) {
                    i += _data.size();
                }
                return _data.at(i);
            }
        }
        inline const Point& at(size_t index) const
        {
            return operator[](index);
        }

        /**
         * Direct access to last added point
         * time and value
         */
        inline double lastTime() const
        {
            return at(0).time;
        }
        inline double lastValue() const
        {
            return at(0).value;
        }

        /**
         * Append given point to the series.
         * Throw error if time is not strictly upper
         * than last stored time
         */
        inline void append(double time, double value)
        {
            if (std::isnan(time) || std::isnan(value)) {
                throw std::logic_error("TimeSeries adding NaN");
            }
            if (_data.size() > 0 && time <= lastTime()) {
                throw std::logic_error("TimeSeries adding past time");
            }

            //Insert the point
            if (_maxSize == (size_t)-1 || _data.size() < _maxSize) {
                _data.push_back({time, value});
            } else {
                _data[_indexEnd] = {time, value};
            }
            _indexEnd++;
            if (_maxSize != (size_t)-1 && _indexEnd >= _maxSize) {
                _indexEnd = 0;
            }

            //Update count/min/max
            _count++;
            if (std::isnan(_min) || value < _min) {
                _min = value;
            }
            if (std::isnan(_max) || value > _max) {
                _max = value;
            }
        }

        /**
         * Load from given input stream.
         * True is returned on sucess.
         * False is returned if given stream is empty.
         * Throw error on malformed input.
         */
        inline bool load(std::istream& is)
        {
            //Extract one line
            std::string line;
            while (
                is.good() && 
                is.peek() != '\n' && 
                is.peek() != EOF
            ) {
                line += is.get();
            }
            is.ignore();
            if (line.length() == 0) {
                return false;
            }

            //Extract name
            size_t pos = 0;
            pos = line.find_first_of(" ", pos);
            if (pos == std::string::npos) {
                throw std::runtime_error(
                    "TimeSeries parsing error: " + line);
            }
            _name = line.substr(0, pos);
            line = line.substr(pos+1);

            //Extract count 
            _count = std::stoi(line, &pos);
            if (pos >= line.length() || line[pos] != ' ') {
                throw std::runtime_error(
                    "TimeSeries parsing error: " + line);
            }
            line = line.substr(pos+1);
            //Extract min
            _min = std::stod(line, &pos);
            if (pos >= line.length() || line[pos] != ' ') {
                throw std::runtime_error(
                    "TimeSeries parsing error: " + line);
            }
            line = line.substr(pos+1);
            //Extract count 
            _max = std::stod(line, &pos);
            if (pos >= line.length() || line[pos] != ' ') {
                throw std::runtime_error(
                    "TimeSeries parsing error: " + line);
            }
            line = line.substr(pos+1);

            //Extract values
            while (line.length() > 0) {
                //Extract time
                double time = std::stod(line, &pos);
                if (pos >= line.length() || line[pos] != ' ') {
                    throw std::runtime_error(
                        "TimeSeries parsing error: " + line);
                }
                line = line.substr(pos+1);
                //Extract value
                double value = std::stod(line, &pos);
                if (pos >= line.length() || line[pos] != ' ') {
                    throw std::runtime_error(
                        "TimeSeries parsing error: " + line);
                }
                line = line.substr(pos+1);
                //Add to series
                append(time, value);
                _count--;
            }
            
            return true;

        }

        /**
         * Dump the time series into given 
         * output stream
         */
        inline void save(std::ostream& os) const
        {
            os << _name << " " 
               << _count << " " 
               << _min <<  " "
               << _max <<  " ";
            for (int i=size()-1;i>=0;i--) {
                os << std::setprecision(10) << at(i).time << " " 
                   << std::setprecision(10) << at(i).value << " ";
            }
            os << std::endl;
        }

    private:

        /**
         * Series name
         */
        std::string _name;

        /**
         * Rolling buffer max size
         * -1 is infinite size
         */
        size_t _maxSize;

        /**
         * Rolling buffer end index
         */
        size_t _indexEnd;

        /**
         * Data points container
         */
        std::vector<Point> _data;

        /**
         * Total number of added points
         */
        size_t _count;

        /**
         * Global minimum and maximum 
         * point value
         */
        double _min;
        double _max;
};

/**
 * Print operator
 */
inline std::ostream& operator<<(std::ostream& os, const TimeSeries& ts)
{
    os << "[" <<ts.name() << "] ";
    if (ts.size() > 0) {
        os << "t=" << ts.lastTime() << " val=" << ts.lastValue() 
            << " (count=" << ts.count() << ") ";
    } else {
        os << "empty" << std::endl;
    }

    return os;
}

}

#endif

