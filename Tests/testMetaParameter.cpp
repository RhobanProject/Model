#include <iostream>
#include <cassert>
#include "TimeSeries/MetaParameter.hpp"

int main()
{
    Leph::MetaParameter param1("param1", 41.0);
    Leph::MetaParameter param2("param2", 42.0);
    Leph::MetaParameter param3("param3", 43.0);
    assert(param1.name() == "param1");
    assert(param2.name() == "param2");
    assert(param3.name() == "param3");
    assert(param1.value() == 41.0);
    assert(param2.value() == 42.0);
    assert(param3.value() == 43.0);

    param2.setMinimum(40.0);
    param3.setMinimum(30.0);
    param3.setMaximum(100.0);
    assert(param2.hasMinimum());
    assert(!param2.hasMaximum());
    assert(param3.hasMinimum());
    assert(param3.hasMaximum());

    param1.setValue(100.0);
    assert(param1.value() == 100.0);
    param2.setValue(0.0);
    assert(param2.value() == 40.0);
    param2.setValue(200.0);
    assert(param2.value() == 200.0);
    
    param3.setValue(40.0);
    assert(param3.value() == 40.0);
    param3.setValue(20.0);
    assert(param3.value() == 30.0);
    param3.setValue(300.0);
    assert(param3.value() == 100.0);

    std::cout << param1 << std::endl;
    std::cout << param2 << std::endl;
    std::cout << param3 << std::endl;
    
    return 0;
}

