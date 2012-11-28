#include "holedetector.h"
#include "holeinspector.h"
#include "mapreader.h"

#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller
{
public:
    Controller();

    HoleDetector* getHoleDetector()
    {
        return hole_detector_;
    }

private:
    MapReader* map_reader_;
    HoleDetector* hole_detector_;
    HoleInspector* hole_inspector_;


};

#endif // CONTROLLER_H
