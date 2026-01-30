#pragma once

#include <memory>
#include <optional>

#include "config/config.h"
#include "types.h"
#include "vehicle/aero/aero.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"

class Vehicle {
    Mass combinedTotalMass;

    Mass nonSuspendedMass;
    Mass suspendedMass;

    float rollCenterHeightFront;
    float rollCenterHeightBack;

    float antiRollStiffnessFront;
    float antiRollStiffnessRear;

    float frontTrackWidth;
    float rearTrackWidth;
    float trackDistance;
    VehicleState state;

    Positioned<Aero> aero;  // i think it should not be Positioned

    // i think that Tires should heve height from witch we would calculate car height and rotations
    WheelData<std::unique_ptr<Tire>> tires;

    // i think that we should move from returning values to influancing the car. Influancing cars
    // state. We should still return values that we influance if we find that it would be more
    // excplicit about changes we are making
    std::array<float, 2> getLatAccAndYawMoment(float tolerance,
                                               const EnvironmentConfig& environmentConfig);
    WheelData<float> calculateSlipAngles(float r, Vec3<float> velocity);
    WheelData<float> staticLoad(float earthAcc);
    float calculateLatAcc(WheelData<float> tireForcesY);
    WheelData<float> distributeForces(float totalForce, float frontDist, float leftDist);
    WheelData<float> totalTireLoads(float latAcc, const EnvironmentConfig& environmentConfig);
    WheelData<float> aeroLoad(const EnvironmentConfig& environmentConfig);
    WheelData<float> loadTransfer(float latAcc);
    VehicleState distributeForces(Positioned<float> Fz, float Tx, float Ty);

   public:
    VehicleState* getState();
    Vehicle(const VehicleConfig& vehicleConfig, const TireConfig& tireConfig);
    Vehicle() = default;
    void calculateYawMomentDiagram(float tolerance, const EnvironmentConfig& environmentConfig);
};
