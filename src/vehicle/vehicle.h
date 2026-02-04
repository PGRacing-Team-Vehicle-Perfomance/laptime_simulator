#pragma once

#include <memory>
#include <optional>

#include "config/config.h"
#include "vehicle/aero/aero.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"

class Vehicle {
    Mass combinedTotalMass;

    Mass combinedNonSuspendedMass;
    Mass combinedSuspendedMass;

    WheelData<float> nonSuspendedMassAtWheels;
    WheelData<float> suspendedMassAtWheels;

    float rollCenterHeightFront;
    float rollCenterHeightBack;

    float antiRollStiffnessFront;
    float antiRollStiffnessRear;

    float frontTrackWidth;
    float rearTrackWidth;
    float trackDistance;
    VehicleState state;

    Positioned<Aero> aero;

    WheelData<Positioned<std::unique_ptr<Tire>>> tires; 
    //WheelData<std::unique_ptr<Tire>> tires;

    std::array<float, 2> getLatAccAndYawMoment(float tolerance,
                                               const EnvironmentConfig& environmentConfig);
    WheelData<float> calculateSlipAngles(float r, Vec3<float> velocity);
    WheelData<float> staticLoad(float earthAcc);
    float calculateLatAcc(WheelData<float> tireForcesY);
    WheelData<float> distributeForces(float totalForce, float frontDist, float leftDist);
    WheelData<float> totalTireLoads(float latAcc, const EnvironmentConfig& environmentConfig);
    WheelData<float> aeroLoad(const EnvironmentConfig& environmentConfig);
    WheelData<float> loadTransfer(float latAcc);
    VehicleState springing(WheelData<float> loads);

   public:
    VehicleState* getState();
    Vehicle(const VehicleConfig& vehicleConfig, const TireConfig& tireConfig);
    Vehicle() = default;
    void calculateYawMomentDiagram(float tolerance, const EnvironmentConfig& environmentConfig);
};
