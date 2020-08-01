#include <mast_finder/libmast_finder.hpp>

namespace iarc2020::mast_locator {

void MastLocator::setSetpoint(mast_finder::Setpoint& setpoint) {
    setpoint_ = setpoint;
}

void MastLocator::setShipcentre(mast_finder::Setpoint& ship_centre) {
    ship_centre_ = ship_centre;
}

void MastLocator::setRadius(float& radius) {
    radius_ = radius;
}

void MastLocator::setNsides(int& n_sides) {
    n_sides_ = n_sides;
}

bool MastLocator::getScoutingdone() {
    return scouting_done_;
}

mast_finder::Setpoint MastLocator::getSetpoint() {
    return setpoint_;
}

void MastLocator::updateSetpoint() {
    setpoint_.x = ship_centre_.x + (radius_ * cos(theta_ - phi_));
    setpoint_.y = ship_centre_.y + (radius_ * sin(theta_ - phi_));
    theta_ += (2 * PI) / n_sides_;
    sides_done_ += 1;
    if (sides_done_ == n_sides_ + 1) {
        scouting_done_ = true;
    }
}

}  // namespace iarc2020::mast_locator
