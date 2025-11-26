#include "simulation/skidpad.h"

#include <cmath>
#include <cstdio>
#include <numbers>

SkidPad::SkidPad(Vehicle& vehicle, SimConfig simConfig, SkidPadConfig skidPadConfig, OptimizationConfig optimizationConfig, SimulationConstants simulationConstants) : vehicle(vehicle), simulationConstants(simulationConstants), simConfig(simConfig), optimizationConfig(optimizationConfig) {
    diameter = trackConfig.minDiameter + trackConfig.diameterOffset;
    radius = diameter / 2;
    trackLength = 2 * std::numbers::pi_v<float> * radius;
}

void SkidPad::run()
{
    float lapTime = 0;
    float acceleration = std::pow(simConfig.startSpeed, 2) / radius;
    float maxIterations = simulationConstants.trackMaxIterations;
    float minIterations = simulationConstants.trackMinIterations;

    for (int i = 0; i < maxIterations; i++)
    {
        float forces = vehicle.getTireForces(simConfig.startSpeed, acceleration, simConfig.airDensity, true);
        float newAcc = forces / vehicle.config.mass;
        float newVelocity = std::sqrt(newAcc * radius);
        float newLapTime = trackLength / newVelocity;

        if (i >= minIterations and abs(newLapTime - lapTime) <= simConfig.errDelta)
        {
            break;
        }

        lapTime = newLapTime;
        acceleration = newAcc;
    }

    float points = calculatePoints(lapTime, trackConfig.historicalBestTime * 1.25, trackConfig.historicalPMax);
    printf("lap time: %f\npoints: %f\n", lapTime, points);
}

float SkidPad::calculatePoints(float time, float tMax, float pMax)
{
    float a = optimizationConfig.pointsCoefficients[0];
    float b = optimizationConfig.pointsCoefficients[1];
    float c = optimizationConfig.pointsCoefficients[2];
    return a * pMax * (std::pow(tMax / time, 2) - 1) / b + c * pMax;
}