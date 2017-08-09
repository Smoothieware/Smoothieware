#include "ExternalSensor.h"

ExternalSensor::ExternalSensor()
{
    m_current_temperature = -1.0F;
}

ExternalSensor::~ExternalSensor()
{
}

bool ExternalSensor::set_temperature(float p_temperature)
{
    m_current_temperature = p_temperature;
    return true;
}

float ExternalSensor::get_temperature()
{
    return m_current_temperature;
}
