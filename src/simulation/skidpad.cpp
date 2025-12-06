#include "simulation/skidpad.h"

#include <cmath>
#include <numbers>

SkidPad::SkidPad(Vehicle &vehicle, SimConfig simConfig, SimulationConstants simulationConstants, SkidPadConfig skidPadConfig) : skidPadConfig(skidPadConfig), Simulation(vehicle, simConfig, simulationConstants)
{
    diameter = skidPadConfig.minDiameter + skidPadConfig.diameterOffset;
    radius = diameter / 2;
    trackLength = 2 * std::numbers::pi_v<float> * radius;
}

float SkidPad::run()
{
    float lapTime = 0;
    float acceleration = std::pow(simConfig.startSpeed, 2) / radius;
    float maxIterations = simulationConstants.trackMaxIterations;
    float minIterations = simulationConstants.trackMinIterations;

    for (int i = 0; i < maxIterations; i++)
    {
        float forces = vehicle.getTireForces(simConfig.startSpeed, acceleration, simConfig, true);
        float newAcc = forces / vehicle.getMass();
        float newVelocity = std::sqrt(newAcc * radius);
        float newLapTime = trackLength / newVelocity;

        if (i >= minIterations and abs(newLapTime - lapTime) <= simConfig.errDelta)
        {
            break;
        }

        lapTime = newLapTime;
        acceleration = newAcc;
    }

    return lapTime;
}

float SkidPad::calculatePoints(float time, const PointsConfig &pointsConfig) const
{
    float a = pointsConfig.pointsCoefficients[0];
    float b = pointsConfig.pointsCoefficients[1];
    float c = pointsConfig.pointsCoefficients[2];
    float pMax = pointsConfig.historicalPMax;
    float tMax = pointsConfig.historicalBestTime * 1.25;
    return a * pMax * (std::pow(tMax / time, 2) - 1) / b + c * pMax;
}
