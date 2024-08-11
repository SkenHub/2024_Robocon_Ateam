#ifndef ANGLE_ACCUMULATOR_H
#define ANGLE_ACCUMULATOR_H

class AngleAccumulator {
public:
    AngleAccumulator();
    void updateAngle(double new_angle);
    double getAccumulatedAngle() const;

private:
    double last_angle_;
    double accumulated_angle_;
    bool is_first_update_;
};

#endif // ANGLE_ACCUMULATOR_H
