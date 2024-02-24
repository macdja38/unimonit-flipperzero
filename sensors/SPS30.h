#ifndef UNITEMP_SPS30
#define UNITEMP_SPS30

#include "../unitemp.h"
#include "../Sensors.h"

extern const SensorType SPS30;

bool unitemp_SPS30_alloc(Sensor* sensor, char* args);
bool unitemp_SPS30_init(Sensor* sensor);
bool unitemp_SPS30_deinit(Sensor* sensor);
UnitempStatus unitemp_SPS30_update(Sensor* sensor);
bool unitemp_SPS30_free(Sensor* sensor);
#endif