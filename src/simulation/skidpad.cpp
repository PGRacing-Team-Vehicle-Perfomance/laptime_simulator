#include "simulation/skidpad.h"

#include <cmath>
#include <numbers>

SkidPad::SkidPad(Vehicle& vehicle, SimConfig simConfig, SimulationConstants simulationConstants,
                 SkidPadConfig skidPadConfig)
    : skidPadConfig(skidPadConfig), Simulation(vehicle, simConfig, simulationConstants) {
    diameter = skidPadConfig.minDiameter + skidPadConfig.diameterOffset;
    radius = diameter / 2;
    trackLength = 2 * std::numbers::pi_v<float> * radius;
}

float SkidPad::run() {
    float lapTime = 0;
    vec2<float> acceleration = {0, static_cast<float>(std::pow(simConfig.startSpeed, 2) / radius)};
    float maxIterations = simulationConstants.trackMaxIterations;
    float minIterations = simulationConstants.trackMinIterations;
    float speed = 100;

    for (int steeringAngle = 0; steeringAngle < 90; steeringAngle++) {
        vehicle.setSteeringAngle(steeringAngle);
        for (int chassisSlipAngle = 0; chassisSlipAngle < 90; chassisSlipAngle++) {
            vehicle.setChassisSlipAngle(chassisSlipAngle);
            vec2<float> diagramPoint = vehicle.getLatAccAndYawMoment(speed);
            // store diagram point as {latAcc, moment, chassisSlipAngle, steeringAngle}
        }
    }
    // plot points
    //  get max data point
    //  calculate time based on data point
    //  return time
}

float SkidPad::calculatePoints(float time, const PointsConfig& pointsConfig) const {
    float a = pointsConfig.pointsCoefficients[0];
    float b = pointsConfig.pointsCoefficients[1];
    float c = pointsConfig.pointsCoefficients[2];
    float pMax = pointsConfig.historicalPMax;
    float tMax = pointsConfig.historicalBestTime * 1.25;
    return a * pMax * (std::pow(tMax / time, 2) - 1) / b + c * pMax;
}
