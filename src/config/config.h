#pragma once

#include "config/configHelper.cpp"
#include "types.h"
#include "vehicle/vehicleHelper.h"

struct EnvironmentConfig {
    float airTemperature = 20;                                                  // [°C]
    float airPressure = 100;                                                    // [kPa]
    float airHumidity = 50;                                                     // [%]
    float airDensity = ::airDensity(airTemperature, airPressure, airHumidity);  // [kg/m³]
    float earthAcc = 9.81;                                                      // [m/s²]
    Vec<ISO8855> wind;  //  amplitude [m/s] , angle [°] 0 = from North
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

    // Toe angles [deg] - positive = toe-in, negative = toe-out
    WheelData<float> toeAngle = {.FL = 0, .FR = 0, .RL = 0, .RR = 0};

    // TODO: Ackermann - implement as one of:
    //   - Ackermann %: 0=parallel, 100=ideal (cot(δ_o)-cot(δ_i)=t/L)
    //   - TOOT [deg] at reference angle [deg]
    //   - Direct inner/outer angles from CAD
    float ackermannPercentage = 0;  // [%] dummy - not implemented

    float cla = 3.7;

    // {0 0} geometric center of front axel
    Position claPosition = {0.75, 0.0, 0.0};  // change to % maby
};

// TODO: split into different configs for simple and pacejka and create implementation based on
// provided
struct TireConfig {
    float scalingFac = 0.75;
    float quadFac = -0.0002;
    float linFac = 1.8284;

    float PCY1 = 1.73993;
    float PDY1 = 3.22192;
    float PDY2 = 0.555412;
    float PDY3 = 8.02593;
    float PEY1 = 0.105808;
    float PEY2 = -0.533932;
    float PEY3 = 0.155092;
    float PEY4 = 0.0330982;
    float PKY1 = 31.8856;
    float PKY2 = 2.01131;
    float PKY3 = -0.0809452;
    float PHY1 = 0.0605117;
    float PHY2 = 0.0486026;
    float PHY3 = -0.1913;
    float PVY1 = 0.132828;
    float PVY2 = 0.0325854;
    float PVY3 = 2.70149;
    float PVY4 = 1.12197;
    float FNOMIN = 3000;
};

struct SkidPadConfig {
    float errDelta = 0.001;
    int maxIterConv = 10;
    float diameter = 1.3;
};
