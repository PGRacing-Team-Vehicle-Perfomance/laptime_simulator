#pragma once

#include "config/configHelper.cpp"
#include "types.h"
#include "vehicle/vehicleHelper.h"

#include <string>

struct EnvironmentConfig {
    float airTemperature = 20;                                                  // [°C]
    float airPressure = 100;                                                    // [kPa]
    float airHumidity = 50;                                                     // [%]
    float airDensity = ::airDensity(airTemperature, airPressure, airHumidity);  // [kg/m³]
    float earthAcc = 9.81;                                                      // [m/s²]
    Vec3f wind = {0, 0, 0};  //  amplitude [m/s] , angle [°] 0 = from North
};

struct VehicleConfig {
    WheelData<float> nonSuspendedMassAtWheels = {.FL = 7.5, .FR = 7.5, .RL = 8, .RR = 8};
    WheelData<float> suspendedMassAtWheels = {.FL = 60, .FR = 60, .RL = 64.5, .RR = 64.5};
    float suspendedMassHeight = 0.33;

    float rollCenterHeightFront = 0.0491;
    float rollCenterHeightBack = 0.0539;

    float frontSpringMotionRatio = 1;
    float frontArbMotionRatio = 1;
    float frontKspring = 1;
    float frontKarb = 1;

    float rearSpringMotionRatio = 1;
    float rearArbMotionRatio = 1;
    float rearKspring = 1;
    float rearKarb = 1;
    // chasiss stiffness would need to be as Nm/deg front rear

    float frontTrackWidth = 1.256;
    float rearTrackWidth = 1.216;
    float trackDistance = 1.53;

    float cla = 3.7;

    // {0 0} geometric center of front axel
    Vec3f claPosition = {0.75, 0.0, 0.0};  // change to % maby
};

// TODO: split into different configs for simple and pacejka and create implementation based on provided
struct TireConfig {
    float scalingFac = 0.75;
    float quadFac = -0.0002;
    float linFac = 1.8284;

    std::string pacejkaParams = "Parameters.txt";
};

struct SkidPadConfig {
    float errDelta = 0.001;
    int maxIterConv = 10;
    float diameter = 1.3;
};
