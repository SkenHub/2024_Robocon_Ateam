#include "encoder.hpp"

Encoder::Encoder(PinName a_pin, PinName b_pin, double diameter, int ppr, int period)
    : encoderA_(a_pin), encoderB_(b_pin), encoderTimer_()
{
    encoderA_.rise(callback(this, &Encoder::interrupt));
    encoderA_.fall(callback(this, &Encoder::interrupt));

    diameter_ = diameter;
    ppr_ = ppr;
    period_ = period;

    count_ = 0;
    limit_ = 0;
    before_rot_ = 0.0;

    encoderTimer_.start();
}

int Encoder::read(void)
{
    return count_;
}

void Encoder::interrupt()
{
    if (encoderA_.read() == encoderB_.read()) {
        count_++;
    } else {
        count_--;
    }

    if (count_ >= 20000 || count_ <= -20000) {
        if (count_ >= 20000) {
            limit_++;
        } else {
            limit_--;
        }
        reset();
    }
}

void Encoder::data(Encoder_data* encoder_data)
{
    if (read() >= 20000 || read() <= -20000) {
        if (read() >= 20000) {
            limit_++;
        } else {
            limit_--;
        }
        reset();
    }
    encoder_data->count_ = read() + limit_ * 20000;
    encoder_data->rot = (encoder_data->count_) / (double)ppr_;
    encoder_data->deg = encoder_data->rot * 360.0;
    encoder_data->distance = encoder_data->rot * (PI * diameter_);

    encoder_data->rps = (double)(encoder_data->rot - before_rot_) / ((double)period_ * 0.001);
    before_rot_ = encoder_data->rot;
    encoder_data->velocity = encoder_data->rps * PI * diameter_;
}

void Encoder::reset()
{
    count_ = 0;
}
