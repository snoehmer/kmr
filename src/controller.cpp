#include "controller.h"

Controller::Controller()
{
    this->map_reader_ = new MapReader();
    this->hole_detector_ = new HoleDetector();
    this->hole_inspector_ = new HoleInspector();
}
