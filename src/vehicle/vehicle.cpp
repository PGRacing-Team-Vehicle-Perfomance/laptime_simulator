#include "vehicle/vehicle.h"
#include "config/config.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"

#include <algorithm>
#include <cmath>

Vehicle::Vehicle(VehicleConfig config) : config(config)
{
    CarWheelBase<bool> isWheelDriven;
    for (size_t i = 0; i < tires.WHEEL_COUNT; i++) isWheelDriven[i] = true;
    
    switch (config.driveType)
    {
    case RWD:
        isWheelDriven.fl = false;
        isWheelDriven.fr = false;
        break;
    case FWD:
        isWheelDriven.rl = false;
        isWheelDriven.rr = false;
        break;
    }

    for (size_t i = 0; i < tires.WHEEL_COUNT; i++)
    {
        tires[i] = Tire(config.tireScalingFactor, config.quadFac, config.linFac, isWheelDriven[i]);
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

float Vehicle::getTireForces(float velocity, float acceleration, const SimConfig& simConfig, bool isLateral)
{
    auto loads = totalTireLoads(velocity, acceleration, simConfig, isLateral);
    float ret = 0;
    
    for (size_t i = 0; i < tires.WHEEL_COUNT; i++)
    {
        ret += tires[i].calculateForce(loads[i], isLateral);
    }

    return ret;
}

CarWheelBase<float> Vehicle::totalTireLoads(float velocity, float acceleration, const SimConfig& simConfig, bool isLateral)
{
    auto static_load = staticLoad(simConfig.earthAcc);
    auto aero = aeroLoad(velocity, simConfig.airDensity);
    auto transfer = loadTransfer(acceleration, isLateral);
    CarWheelBase<float> ret;
    for (size_t i = 0; i < tires.WHEEL_COUNT; i++)
    {
        ret[i] = std::max(0.f, static_load[i] + aero[i] + transfer[i]);
    }
    return ret;
}

CarWheelBase<float> Vehicle::staticLoad(float earthAcc)
{
    return distributeForces(config.mass * earthAcc, config.frontWeightDist, config.leftWeightDist);
}

CarWheelBase<float> Vehicle::distributeForces(float totalForce, float frontDist, float leftDist)
{
    CarWheelBase<float> forces;
    forces.fl = totalForce * frontDist * leftDist;
    forces.fr = totalForce * frontDist * (1 - leftDist);
    forces.rl = totalForce * (1 - frontDist) * leftDist;
    forces.rr = totalForce * (1 - frontDist) * (1 - leftDist);
    return forces;
}

CarWheelBase<float> Vehicle::aeroLoad(float velocity, float airDensity)
{
    float totalForce = 0.5 * config.cla * airDensity * std::pow(velocity, 2);
    return distributeForces(totalForce, config.frontAeroDist, config.leftAeroDist);
}

CarWheelBase<float> Vehicle::loadTransfer(float acceleration, bool isLateral)
{
    CarWheelBase<float> loads;
    float moment = acceleration * config.mass * config.cogHeight;
    if (isLateral)
    {
        float front = moment * config.frontWeightDist / config.frontTrackWidth;
        float rear = moment * (1 - config.frontWeightDist) / config.rearTrackWidth;
        loads.fl = -front;
        loads.fr = front;
        loads.rl = -rear;
        loads.rr = rear;
        return loads;
    }
    float transfer = moment / config.wheelbase;
    float left = config.leftWeightDist;
    
    loads.fl = -transfer * left;
    loads.fr = -transfer * (1 - left);
    loads.rl = transfer * left;
    loads.rr = transfer * (1 - left);
    
    return loads;
}
