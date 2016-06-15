#include <fstream>
#include <iomanip>
#include <stdexcept>
#include "Utils/FileVector.h"

namespace Leph {

void WriteVector(
    const std::string& filename, 
    const Eigen::VectorXd& vect)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error(
            "WriteVector unable to open file: " 
            + filename);
    }
    file << vect.size();
    for (size_t i=0;i<(size_t)vect.size();i++) {
        file << " " << std::setprecision(15) << vect(i);
    }
    file << std::endl;
    file.close();
}

Eigen::VectorXd ReadVector(
    const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error(
            "ReadVector unable to open file: " 
            + filename);
    }
    size_t size;
    file >> size;
    Eigen::VectorXd vect(size);
    for (size_t i=0;i<size;i++) {
        double value;
        file >> value;
        vect(i) = value;
    }
    file.close();
    return vect;
}

}

