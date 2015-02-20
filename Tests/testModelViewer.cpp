#include <iostream>
#include "Viewer/ModelViewer.hpp"

int main()
{
    Leph::ModelViewer viewer(800, 600);

    while (viewer.update()) {
        //TODO
    }

    return 0;
}

