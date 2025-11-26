#include "vehicle/vehicle.h"
#include "config/config.h"
#include "vehicle/tire/tire.h"

#include <cstdio>
#include <array>
#include <algorithm>
#include <cmath>

template <typename T>
int indexOf(const std::vector<T> &vec, const T &value)
{
    auto it = std::find(vec.begin(), vec.end(), value);
    return (it == vec.end()) ? -1 : std::distance(vec.begin(), it);
}

Vehicle::Vehicle(VehicleConfig config, float tireScalingFactor) : config(config)
{
    for (auto wheel : WHEELS)
    {
        tires.emplace(wheel, Tire(tireScalingFactor, config.quadFac, config.linFac));
    }

    /*
    if (not all(wheel in self.WHEELS for wheel in config.driven_wheels))
    {
        abort();
        // raise ValueError(f'Driven wheels {config.driven_wheels} must be a subset of {self.WHEELS}')
    }
    */
    /*
    rpms, torques = zip(*config.torque_curve)
    torque_interp = lru_cache(maxsize=1000)(interp1d(rpms, torques, kind='linear', fill_value='extrapolate'))
    self.gear_speed_ranges = [
        (
            (config.idle_rpm * np.pi * config.wheel_radius) / (30 * ratio * config.final_drive_ratio),
            (config.redline_rpm * np.pi * config.wheel_radius) / (30 * ratio * config.final_drive_ratio),
        )
        for ratio in config.gear_ratios
    ]
    */
}

float Vehicle::getTireForces(float velocity, float acceleration, float airDensity, bool isLateral)
{
    SimConfig simConfig;
    simConfig.airDensity = airDensity;
    auto loads = totalTireLoads(velocity, acceleration, simConfig, isLateral);
    std::vector<std::string> wheels = isLateral ? WHEELS : config.drivenWheels;

    float ret = 0;
    for (auto wheel : wheels)
    {
        ret += tires[wheel].calculateForce(loads[indexOf(WHEELS, wheel)]);
    }

    return ret;
}

std::array<float, 4> Vehicle::totalTireLoads(float velocity, float acceleration, SimConfig simConfig, bool isLateral)
{
    auto static_load = staticLoad(simConfig);
    auto aero = aeroLoad(velocity, simConfig);
    auto transfer = loadTransfer(acceleration, isLateral);
    std::array<float, 4> ret;
    for (int i = 0; i < 4; i++)
    {
        ret[i] = std::max(0.f, static_load[i] + aero[i] + transfer[i]);
    }
    return ret;
}

std::array<float, 4> Vehicle::staticLoad(SimConfig simConfig)
{
    return distributeForces(config.mass * simConfig.earthAcc, config.frontWeightDist, config.leftWeightDist);
}

std::array<float, 4> Vehicle::distributeForces(float totalForce, float frontDist, float leftDist)
{
    float lf = totalForce * frontDist * leftDist;
    float rf = totalForce * frontDist * (1 - leftDist);
    float lr = totalForce * (1 - frontDist) * leftDist;
    float rr = totalForce * (1 - frontDist) * (1 - leftDist);
    return {lf, rf, lr, rr};
}

std::array<float, 4> Vehicle::aeroLoad(float velocity, SimConfig simConfig)
{
    float totalForce = 0.5 * config.cla * simConfig.airDensity * std::pow(velocity, 2);
    return distributeForces(totalForce, config.frontAeroDist, config.leftAeroDist);
}

std::array<float, 4> Vehicle::loadTransfer(float acceleration, bool isLateral)
{
    float moment = acceleration * config.mass * config.cogHeight;
    if (isLateral)
    {
        float front = moment * config.frontWeightDist / config.frontTrackWidth;
        float rear = moment * (1 - config.frontWeightDist) / config.rearTrackWidth;
        return {-front, front, -rear, rear};
    }
    float transfer = moment / config.wheelbase;
    float left = config.leftWeightDist;
    return {-transfer * left, -transfer * (1 - left), transfer * left, transfer * (1 - left)};
}
