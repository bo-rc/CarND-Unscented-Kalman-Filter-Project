#include "Sensor.h"

Sensor::Sensor():
    name("generic"),
    is_initialized(false),
    is_active(false),
    has_new_data(false),
    data_dim(1)
{

}

Sensor::~Sensor()
{
    // nothing to do
}
