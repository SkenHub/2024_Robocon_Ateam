#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "mbed.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

struct Encoder_data {
    int count_;
    double rot;
    double deg;
    double distance;
    double rps;
    double velocity;
};

class Encoder {
public:
    Encoder(PinName a_pin, PinName b_pin, double diameter = 100, int ppr = 8192, int period = 1);
    int read(void);
    void reset();
    void data(Encoder_data* encoder_data);

private:
    void interrupt();

    InterruptIn encoderA_;
    DigitalIn encoderB_;
    Timer encoderTimer_;
    
    double diameter_;
    int ppr_;
    int period_;

    int count_;
    int limit_;
    double before_rot_;
};

#endif // ENCODER_HPP
