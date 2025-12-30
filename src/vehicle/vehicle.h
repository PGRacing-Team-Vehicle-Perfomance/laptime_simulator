#pragma once

#include "config/config.h"
#include "vehicle/body.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"

class Vehicle {
    struct {
        std::vector<std::unique_ptr<Body>> chacieElements;
        std::vector<std::unique_ptr<AeroElement>> aeroElements;
        std::vector<std::unique_ptr<Body>> wheelAssemblyElements;
        std::vector<std::unique_ptr<Body>> suspentionElements;
        CarWheelBase<std::unique_ptr<Tire>> tires;
        Body engine;
    } parts;

    struct {
        vec3<float> roll;
        vec3<float> pitch;
        vec3<float> yaw;
    } degOfFreedomCenter;

    struct {
        vec3<float> linear;
        vec3<float> angular;
    } velocity;

    const float rollCenterHeightFront;
    const float rollCenterHeightBack;
    const float antiRollStiffnessFront = 15000.0;
    const float antiRollStiffnessRear = 15000.0;
    const float frontTrackWidth = 1.256;
    const float rearTrackWidth = 1.216;

    Body combinedTotalMass;
    Body combinedSuspendedMass;
    Body combinedUnsuspendedMass;

    float combinedCla;
    vec3<float> combinedClaPosition;

    float cda = 1.3;
    float crr = 0.015;
    float wheelbase = 1.53;

    VehicleConfig config;
    CarWheelBase<std::unique_ptr<Tire>> tires;

    CarWheelBase<float> distributeForces(float totalForce, float frontDist, float leftDist);
    CarWheelBase<float> totalTireLoads(float velocity, vec2<float> acceleration,
                                       const SimConfig& simConfig);
    CarWheelBase<float> staticLoad(float earthAcc);
    CarWheelBase<float> aeroLoad(float velocity, float airDensity);
    CarWheelBase<float> loadTransfer(vec2<float> acceleration);
    float getEngineTorque(float rpm);
    float getWheelTorque(float engine_torque, int gear);

   public:
    Vehicle(VehicleConfig config);
    float getTireForces(float startSpeed, vec2<float> acceleration, const SimConfig& simConfig);
    float speedToRpm(float speed_ms, int gear);
    float getPowerThrust(float speed_ms, int gear);

    float getTotalMass();
    float getCRR();
    float getCDA();
    float getMaxTorqueRpm();
    unsigned int getGearCount();
    float getShiftTime();
};
