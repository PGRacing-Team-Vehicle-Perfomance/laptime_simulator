#pragma once

#include <cmath>
#include <optional>

inline float AirDensity(float airTemperature, float airPressure,
                        std::optional<float> relativeHumidity) {
    float humidity = relativeHumidity.value_or(50.0f);
    constexpr float R_DRY = 287.058f;    // specific gas constant for dry air (J/(kg·K))
    constexpr float R_VAPOR = 461.495f;  // specific gas constant for water vapor (J/(kg·K))

    float tempKelvin = airTemperature + 273.15f;

    // saturation vapor pressure (Magnus formula)
    float pSat = 610.78f * expf((17.27f * airTemperature) / (airTemperature + 237.3f));

    // partial pressures
    float pVapor = (humidity / 100.0f) * pSat;
    float pDry = airPressure - pVapor;
    float airDensity = (pDry / (R_DRY * tempKelvin)) + (pVapor / (R_VAPOR * tempKelvin));

    return airDensity;
}
