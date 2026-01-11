#pragma once

#include <cmath>

#include "config/configHelper.cpp"
#include "vehicle/vehicleHelper.h"

struct EnvironmentConfig {
    float airTemperature = 20;                                                // [°C]
    float airPressure = 100;                                                  // [kPa]
    float airHumidity = NAN;                                                  // [%]
    float airDensity = AirDensity(airTemperature, airPressure, airHumidity);  // [kg/m³]
    float earthAcc = 9.81;                                                    // [m/s²]
    polarVector wind = {.amplitude = 0,
                        .angle = Angle(0)};  //  amlitude [m/s] , angle [°] 0 = from North
};

struct VehicleConfig {
    // {0 0} geometric center of front axel
    Body nonSuspendedMass = {0.0, {0, 0, 0}};
    Body suspendedMass = {280.0, {0, 0, 0}};

    float rollCenterHeightFront = 0.33;
    float rollCenterHeightBack = 0.33;

    float antiRollStiffnessFront = 15000.0;
    float antiRollStiffnessRear = 15000.0;

    float frontTrackWidth = 1.256;
    float rearTrackWidth = 1.216;
    float trackDistance = 2;

    float cla = 4.3;
    vec2<float> claPosition = {0.5, 0.5};
};

struct SkidPadConfig {
    float errDelta = 0.001;
    int maxIterConv = 10;
    float diameter = 1.3;
};
