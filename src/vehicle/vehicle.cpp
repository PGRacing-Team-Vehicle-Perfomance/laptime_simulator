#include "vehicle/vehicle.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>

#include "config/config.h"
#include "vehicle/tire/tire.h"
#include "vehicle/tire/tireSimple.h"
#include "vehicle/vehicleHelper.h"

Vehicle::Vehicle(VehicleConfig config) : config(config) {
    CarWheelBase<bool> isWheelDriven;
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        isWheelDriven[i] = true;
    }

    switch (config.driveType) {
        case CarAcronyms::RWD:
            isWheelDriven[CarAcronyms::FL] = false;
            isWheelDriven[CarAcronyms::FR] = false;
            break;
        case CarAcronyms::FWD:
            isWheelDriven[CarAcronyms::RL] = false;
            isWheelDriven[CarAcronyms::RR] = false;
            break;
    }

    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        tires[i] = std::make_unique<TireSimple>(config.tireScalingFactor, config.quadFac,
                                                config.linFac, isWheelDriven[i]);
    }
}

float Vehicle::getTireForces(float velocity, float acceleration, const SimConfig &simConfig,
                             bool isLateral) {
    auto loads = totalTireLoads(velocity, acceleration, simConfig, isLateral);
    float ret = 0;

    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        if (isLateral) {
            ret += tires[i]->getLateralForce(loads[i]);
        } else {
            ret += tires[i]->getLongitudinalForce(loads[i]);
        }
    }

    return ret;
}

CarWheelBase<float> Vehicle::totalTireLoads(float velocity, float acceleration,
                                            const SimConfig &simConfig, bool isLateral) {
    auto static_load = staticLoad(simConfig.earthAcc);
    auto aero = aeroLoad(velocity, simConfig.airDensity);
    auto transfer = loadTransfer(acceleration, isLateral);
    CarWheelBase<float> ret;
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        ret[i] = std::max(0.f, static_load[i] + aero[i] + transfer[i]);
    }
    return ret;
}

CarWheelBase<float> Vehicle::staticLoad(float earthAcc) {
    return distributeForces(config.mass * earthAcc, config.frontWeightDist, config.leftWeightDist);
}

CarWheelBase<float> Vehicle::distributeForces(float totalForce, float frontDist, float leftDist) {
    CarWheelBase<float> forces;
    forces[CarAcronyms::FL] = totalForce * frontDist * leftDist;
    forces[CarAcronyms::FR] = totalForce * frontDist * (1 - leftDist);
    forces[CarAcronyms::RL] = totalForce * (1 - frontDist) * leftDist;
    forces[CarAcronyms::RR] = totalForce * (1 - frontDist) * (1 - leftDist);
    return forces;
}

CarWheelBase<float> Vehicle::aeroLoad(float velocity, float airDensity) {
    float totalForce = 0.5 * config.cla * airDensity * std::pow(velocity, 2);
    return distributeForces(totalForce, config.frontAeroDist, config.leftAeroDist);
}

CarWheelBase<float> Vehicle::loadTransfer(float acceleration, bool isLateral) {
    CarWheelBase<float> loads;
    float moment = acceleration * config.mass * config.cogHeight;
    if (isLateral) {
        float front = moment * config.frontWeightDist / config.frontTrackWidth;
        float rear = moment * (1 - config.frontWeightDist) / config.rearTrackWidth;
        loads[CarAcronyms::FL] = -front;
        loads[CarAcronyms::FR] = front;
        loads[CarAcronyms::RL] = -rear;
        loads[CarAcronyms::RR] = rear;
        return loads;
    }
    float transfer = moment / config.wheelbase;
    float left = config.leftWeightDist;

    loads[CarAcronyms::FL] = -transfer * left;
    loads[CarAcronyms::FR] = -transfer * (1 - left);
    loads[CarAcronyms::RL] = transfer * left;
    loads[CarAcronyms::RR] = transfer * (1 - left);

    return loads;
}

float Vehicle::getMass() { return config.mass; }
float Vehicle::getCRR() { return config.crr; }
float Vehicle::getCDA() { return config.cda; }
float Vehicle::getMaxTorqueRpm() { return config.maxTorqueRpm; }
unsigned int Vehicle::getGearCount() { return config.gearRatios.size(); }
float Vehicle::getShiftTime() { return config.shiftTime; }

float Vehicle::speedToRpm(float speed_ms, int gear) {
    if (gear < 0 || gear >= config.gearRatios.size()) {
        return config.idleRpm;
    }
    float ratio = config.gearRatios[gear] * config.finalDriveRatio;
    float rpm = (speed_ms / config.wheelRadius) * (30 / std::numbers::pi_v<float>)*ratio;
    return std::max(static_cast<float>(config.idleRpm),
                    std::min(rpm, static_cast<float>(config.redlineRpm)));
}

float Vehicle::getPowerThrust(float speed_ms, int gear) {
    float rpm = speedToRpm(speed_ms, gear);
    float torque = getEngineTorque(rpm);
    float wheelTorque = getWheelTorque(torque, gear);
    float thrust = wheelTorque / config.wheelRadius;
    return std::max(thrust, 0.f);
}

float Vehicle::getWheelTorque(float engine_torque, int gear) {
    if (gear < 0 || gear >= config.gearRatios.size()) {
        return 0.0;
    }
    return engine_torque * config.gearRatios[gear] * config.finalDriveRatio;
}

float Vehicle::getEngineTorque(float rpm) {
    auto it = std::lower_bound(config.torqueCurve.begin(), config.torqueCurve.end(), rpm,
                               [](auto &p, float v) { return p.first < v; });
    if (it == config.torqueCurve.begin()) return it->second;
    if (it == config.torqueCurve.end()) return (it - 1)->second;

    auto &[x0, y0] = *(it - 1);
    auto &[x1, y1] = *it;
    return y0 + (rpm - x0) * (y1 - y0) / (x1 - x0);
}
