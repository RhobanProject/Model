#include <iostream>
#include <cassert>
#include <sstream>
#include "TimeSeries/TimeSeries.hpp"

int main()
{
    Leph::TimeSeries series1("test1");
    Leph::TimeSeries series2("test2", 4);

    assert(series1.name() == "test1");
    assert(series2.name() == "test2");
    assert(series1.size() == 0);
    assert(series2.size() == 0);
    assert(series1.count() == 0);
    assert(series2.count() == 0);

    series1.append(1.0, 1.5);
    series2.append(1.0, 1.5);
    
    assert(series1.name() == "test1");
    assert(series2.name() == "test2");
    assert(series1.size() == 1);
    assert(series2.size() == 1);
    assert(series1.count() == 1);
    assert(series2.count() == 1);
    assert(series1.min() == 1.5);
    assert(series1.max() == 1.5);
    assert(series2.min() == 1.5);
    assert(series2.max() == 1.5);
    assert(series1.lastTime() == 1.0);
    assert(series1.lastValue() == 1.5);
    assert(series2.lastTime() == 1.0);
    assert(series2.lastValue() == 1.5);
    assert(series1[0].time == 1.0);
    assert(series1[0].value == 1.5);
    assert(series2[0].time == 1.0);
    assert(series2[0].value == 1.5);
    assert(series1.timeMin() == 1.0);
    assert(series1.timeMax() == 1.0);
    assert(series2.timeMin() == 1.0);
    assert(series2.timeMax() == 1.0);
    
    series1.append(2.0, 2.5);
    series2.append(2.0, 2.5);
    series1.append(3.0, 3.5);
    series2.append(3.0, 3.5);
    
    assert(series1.size() == 3);
    assert(series2.size() == 3);
    assert(series1.count() == 3);
    assert(series2.count() == 3);
    assert(series1.min() == 1.5);
    assert(series1.max() == 3.5);
    assert(series2.min() == 1.5);
    assert(series2.max() == 3.5);
    assert(series1.lastTime() == 3.0);
    assert(series1.lastValue() == 3.5);
    assert(series2.lastTime() == 3.0);
    assert(series2.lastValue() == 3.5);
    assert(series1[0].time == 3.0);
    assert(series1[0].value == 3.5);
    assert(series2[0].time == 3.0);
    assert(series2[0].value == 3.5);
    assert(series1[2].time == 1.0);
    assert(series1[2].value == 1.5);
    assert(series2[2].time == 1.0);
    assert(series2[2].value == 1.5);
    assert(series1.timeMin() == 1.0);
    assert(series1.timeMax() == 3.0);
    assert(series2.timeMin() == 1.0);
    assert(series2.timeMax() == 3.0);
    assert(series1.get(2.5) == 3.0);
    assert(series2.get(2.5) == 3.0);
    assert(series1.get(3.0) == 3.5);
    assert(series2.get(3.0) == 3.5);
    
    series1.append(4.0, 4.5);
    series2.append(4.0, 4.5);
    series1.append(5.0, 5.5);
    series2.append(5.0, 5.5);
    
    assert(series1.size() == 5);
    assert(series2.size() == 4);
    assert(series1.count() == 5);
    assert(series2.count() == 5);
    assert(series1.min() == 1.5);
    assert(series1.max() == 5.5);
    assert(series2.min() == 1.5);
    assert(series2.max() == 5.5);
    assert(series1.lastTime() == 5.0);
    assert(series1.lastValue() == 5.5);
    assert(series2.lastTime() == 5.0);
    assert(series2.lastValue() == 5.5);
    assert(series1[0].time == 5.0);
    assert(series1[0].value == 5.5);
    assert(series2[0].time == 5.0);
    assert(series2[0].value == 5.5);
    assert(series1[4].time == 1.0);
    assert(series1[4].value == 1.5);
    assert(series2[3].time == 2.0);
    assert(series2[3].value == 2.5);
    assert(series1.timeMin() == 1.0);
    assert(series1.timeMax() == 5.0);
    assert(series2.timeMin() == 2.0);
    assert(series2.timeMax() == 5.0);
    assert(series1.get(4.5) == 5.0);
    assert(series2.get(4.5) == 5.0);
    assert(series1.get(4.0) == 4.5);
    assert(series2.get(4.0) == 4.5);
    std::cout << series2 << std::endl;

    std::ostringstream oss;
    series1.save(oss);
    series1.save(oss);
    std::cout << oss.str();
    std::cout << std::endl;

    Leph::TimeSeries series3;
    Leph::TimeSeries series4;
    Leph::TimeSeries series5;
    std::istringstream iss(oss.str());
    series3.load(iss);
    series4.load(iss);
    series5.load(iss);

    assert(series3.size() == 5);
    assert(series3.count() == 5);
    assert(series3.min() == 1.5);
    assert(series3.max() == 5.5);
    assert(series3.lastTime() == 5.0);
    assert(series3.lastValue() == 5.5);
    assert(series3[0].time == 5.0);
    assert(series3[0].value == 5.5);
    assert(series3[4].time == 1.0);
    assert(series3[4].value == 1.5);
    assert(series3.timeMin() == 1.0);
    assert(series3.timeMax() == 5.0);
    assert(series3.get(4.5) == 5.0);

    assert(series5.size() == 0);
    
    series3.save(std::cout);
    series4.save(std::cout);
    series5.save(std::cout);
    
    series4.clear();
    assert(series4.size() == 0);
    assert(series4.count() == 0);

    return 0;
}

