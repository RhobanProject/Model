#include <stdexcept>
#include "Utils/Chrono.hpp"

namespace Leph {
        
Chrono::Chrono() :
    _timePoints(),
    _durations()
{
}
        
void Chrono::clear()
{
    _timePoints.clear();
    _durations.clear();
}
        
void Chrono::start(const std::string& name)
{
    _timePoints[name] = std::chrono::system_clock::now();
}
        
void Chrono::stop(const std::string& name, 
    const std::string& reference)
{
    TimePoint now = std::chrono::system_clock::now();
    
    if (_timePoints.count(reference) == 0) {
        throw std::logic_error(
            "Chrono not started: " + reference);
    }

    Duration elapsed = now - _timePoints.at(reference);
    _durations[name] = elapsed;
}
        
double Chrono::getDurationSec(const std::string& name) const
{
    if (_durations.count(name) == 0) {
        throw std::logic_error(
            "Chrono unknown duration: " + name);
    }

    return std::chrono::duration_cast<
        std::chrono::duration<double, std::ratio<1>>>(
            _durations.at(name)).count();
}
double Chrono::getDurationMs(const std::string& name) const
{
    if (_durations.count(name) == 0) {
        throw std::logic_error(
            "Chrono unknown duration: " + name);
    }

    return std::chrono::duration_cast<
        std::chrono::duration<double, std::ratio<1, 1000>>>(
            _durations.at(name)).count();
}
        
void Chrono::print(std::ostream& os) const
{
    for (const auto& d : _durations) {
        os << "[" << d.first << "] " 
            << getDurationMs(d.first) << "ms" << std::endl;
    }
}

}

