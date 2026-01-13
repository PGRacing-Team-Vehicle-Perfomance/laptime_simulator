#pragma once

#include <cmath>

inline float airDensity(float airTemperature = 20, float airPressure = 100,
                        float relativeHumidity = 50) {
    constexpr float R_DRY = 287.058f;    // specific gas constant for dry air (J/(kg·K))
    constexpr float R_VAPOR = 461.495f;  // specific gas constant for water vapor (J/(kg·K))

    float tempKelvin = airTemperature + 273.15f;

    // saturation vapor pressure (Magnus formula)
    float pSat = 610.78f * expf((17.27f * airTemperature) / (airTemperature + 237.3f));

    // partial pressures
    float pVapor = (relativeHumidity / 100.0f) * pSat;
    float pDry = airPressure - pVapor;
    float airDensity = (pDry / (R_DRY * tempKelvin)) + (pVapor / (R_VAPOR * tempKelvin));

    return airDensity;
}
