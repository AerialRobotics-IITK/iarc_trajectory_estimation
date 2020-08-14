#pragma once

#include <iostream>
#include <mast_finder/Setpoint.h>
#include <math.h>

#define PI 3.14159265358979323846

namespace iarc2020::mast_locator {

class MastLocator {
  public:
    void setSetpoint(mast_finder::Setpoint& setpoint);
    void setShipcentre(mast_finder::Setpoint& ship_centre);
    void setRadius(float& radius);
    void setNsides(int& n_sides);
    void updateSetpoint();
    bool getScoutingDone();
    int getSidesDone();
    mast_finder::Setpoint getSetpoint();

  private:
    mast_finder::Setpoint setpoint_;
    mast_finder::Setpoint ship_centre_;
    float radius_;
    int n_sides_;
    double theta_ = 0.0;
    double phi_ = PI;
    bool scouting_done_ = false;
    int sides_done_ = 0;
};

}  // namespace iarc2020::mast_locator
