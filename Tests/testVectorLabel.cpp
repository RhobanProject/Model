#include <iostream>
#include <fstream>
#include <cassert>
#include "Types/VectorLabel.hpp"

int main()
{
    Leph::VectorLabel vect1(4);
    std::cout << vect1 << std::endl;
    Leph::VectorLabel vect2({"l1", "l2", "l4", "l3"});
    std::cout << vect2 << std::endl;
    Leph::VectorLabel vect3(Eigen::VectorXd(3));
    std::cout << vect3 << std::endl;
    Leph::VectorLabel vect4({"l3", "l2", "l1"}, Eigen::VectorXd(3));
    std::cout << vect4 << std::endl;

    std::cout << Leph::VectorLabel::mergeUnion(vect1, vect2) << std::endl;
    std::cout << Leph::VectorLabel::mergeUnion(vect1, vect1) << std::endl;
    std::cout << (vect1 & vect2) << std::endl;

    std::cout << "Vect2" << std::endl;
    std::cout << vect2 << std::endl;
    std::cout << "Vect4" << std::endl;
    std::cout << vect4 << std::endl;
    std::cout << "Inter 2-4" << std::endl;
    std::cout << Leph::VectorLabel::mergeInter(vect2, vect4) << std::endl;
    std::cout << "Inter 4-4" << std::endl;
    std::cout << Leph::VectorLabel::mergeInter(vect4, vect4) << std::endl;
    
    Leph::VectorLabel vect6(
        "test label 1", 1.0,
        "test label 2", 2.0);
    assert(vect6.exist("test label 1"));
    assert(vect6("test label 1") == 1.0);
    assert(!vect6.exist("test label 3"));
    assert(!vect6.exist("test label 4"));
    vect6.append(
        "test label 3", 3.0,
        "test label 4", 4.0);
    assert(vect6.exist("test label 3"));
    assert(vect6.exist("test label 4"));
    std::cout << vect6 << std::endl;
    
    assert(vect1.vect().size() == 4);
    assert(vect1.size() == 4);
    vect1.vect()(0) = 1.0;
    assert(vect1.vect()(0) == 1.0);
    std::cout << vect1 << std::endl;

    assert(vect2.labels().size() == 4);
    assert(vect2.getLabel(0) == "l1");
    assert(vect2.getIndex("l1") == 0);

    vect2("l1") = 0.0;
    assert(vect2("l1") == 0.0);
    vect2("l1") = 1.0;
    assert(vect2("l1") == 1.0);
    std::cout << vect2 << std::endl;
    
    vect2(1) = 0.0;
    assert(vect2(1) == 0.0);
    vect2(1) = 2.0;
    assert(vect2(1) == 2.0);
    std::cout << vect2 << std::endl;

    vect2.writeToCSV();
    std::ofstream file("/tmp/testVectorLabel.csv");
    vect2.writeToCSV(file);

    vect2("l1") = 666;
    std::string csvStr = "#t1 t2 l3\n0.1 0.2 42.0\n\n";
    vect2.readFromCSV(csvStr);
    vect2.writeToCSV(file);
    std::cout << vect2 << std::endl;
    file.close();
    
    std::ifstream fileIn("/tmp/testVectorLabel.csv");
    while (vect2.readFromCSV(fileIn)) {
        std::cout << vect2 << std::endl;
    }
    fileIn.close();

    std::cout << "Vect2" << std::endl;
    std::cout << vect2 << std::endl;
    std::cout << "Vect3" << std::endl;
    vect3.append("l3", 69);
    std::cout << vect3 << std::endl;
    std::cout << "Vect3 union 2" << std::endl;
    vect3.mergeUnion(vect2);
    std::cout << vect3 << std::endl;
    std::cout << "Vect2 inter 3" << std::endl;
    vect3("l2") = 100.0;
    vect2.mergeInter(vect3);
    std::cout << vect2 << std::endl;
    
    vect2 += vect2;
    std::cout << vect2 << std::endl;
    vect2 *= 0.5;
    std::cout << vect2 << std::endl;
    vect2 *= vect2;
    std::cout << vect2 << std::endl;

    return 0;
}

