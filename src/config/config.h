#pragma once

#include "config/configHelper.cpp"
#include "types.h"

struct EnvironmentConfig {
    float airTemperature = 20;                                                  // [°C]
    float airPressure = 100;                                                    // [kPa]
    float airHumidity = 50;                                                     // [%]
    float airDensity = ::airDensity(airTemperature, airPressure, airHumidity);  // [kg/m³]
    float earthAcc = 9.81;                                                      // [m/s²]
    Vec3f wind = {0, 0, 0};  //  amplitude [m/s] , angle [°] 0 = from North
};

struct VehicleConfig {
    // {0 0} geometric center of front axel
    Mass nonSuspendedMass = {0.0, {0, 0, 0}};
    Mass suspendedMass = {280.0, {0, 0, 0}};

    float rollCenterHeightFront = 0.33;
    float rollCenterHeightBack = 0.33;

    float antiRollStiffnessFront = 15000.0;
    float antiRollStiffnessRear = 15000.0;

    float frontTrackWidth = 1.256;
    float rearTrackWidth = 1.216;
    float trackDistance = 2;

    float cla = 4.3;
    Vec3f claPosition = {0.5, 0.5, 0.0};
};

struct TireConfig {
    float scalingFac;
    float quadFac;
    float linFac;
};

struct SkidPadConfig {
    float errDelta = 0.001;
    int maxIterConv = 10;
    float diameter = 1.3;
};
