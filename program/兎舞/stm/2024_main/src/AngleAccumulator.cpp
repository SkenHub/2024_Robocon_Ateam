#include "AngleAccumulator.hpp"

AngleAccumulator::AngleAccumulator()
    : last_angle_(0), accumulated_angle_(0), is_first_update_(true) {}

void AngleAccumulator::updateAngle(double new_angle) {
    if (is_first_update_) {
        last_angle_ = new_angle;
        is_first_update_ = false;
    } else {
        double delta_angle = new_angle - last_angle_;

        if (delta_angle > 180) {
            delta_angle -= 360;
        } else if (delta_angle < -180) {
            delta_angle += 360;
        }

        accumulated_angle_ += delta_angle;
        last_angle_ = new_angle;
    }
}

double AngleAccumulator::getAccumulatedAngle() const {
    return accumulated_angle_;
}
