#ifndef EXTERNALSENSOR_H
#define EXTERNALSENSOR_H

#include "TempSensor.h"


class ExternalSensor : public TempSensor
{
public:

    ExternalSensor();
    ~ExternalSensor();

    // Return temperature in degrees Celsius.
    float get_temperature();

    // Set and store the internal temperature
    bool set_temperature(float p_temperature);

private:
    float m_current_temperature;

};

#endif
