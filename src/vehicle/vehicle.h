#pragma once

#include <array>
#include <memory>
#include <optional>
#include <vector>

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/aero/aero.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"

class Vehicle {
    Mass<> combinedTotalMass;

    Mass<> combinedNonSuspendedMass;
    Mass<> combinedSuspendedMass;

    WheelData<float> nonSuspendedMassAtWheels;
    WheelData<float> suspendedMassAtWheels;

    float rollCenterHeightFront;
    float rollCenterHeightBack;

    float antiRollStiffnessFront;
    float antiRollStiffnessRear;

    float frontTrackWidth;
    float rearTrackWidth;
    float trackDistance;

    WheelData<Alpha<>> toeAngle;
    float ackermannPercentage;

    VehicleState state;

    Positioned<Aero> aero;

    WheelData<Positioned<std::unique_ptr<Tire<SAE>>>> tires;

    std::array<float, 2> getLatAccAndYawMoment(float tolerance, int maxIterations,
                                               const EnvironmentConfig& environmentConfig);
    WheelData<Alpha<>> calculateSlipAngles(Alpha<> yawRate, Vec<> velocity);
    WheelData<Alpha<>> calculateSteeringAngles();
    WheelData<float> staticLoad(float earthAcc);
    Y<> calculateLatAcc(const WheelData<X<>>& tireForcesX,
                               const WheelData<Y<>>& tireForcesY);
    WheelData<float> distributeForces(float totalForce, float frontDist, float leftDist);
    WheelData<float> totalTireLoads(Y<> latAcc, const EnvironmentConfig& environmentConfig);
    WheelData<float> aeroLoad(const EnvironmentConfig& environmentConfig);
    WheelData<float> loadTransfer(Y<> latAcc);
    WheelData<Y<>> getVehicleFyFromTireForces(const WheelData<X<>>& tireFx,
                                                     const WheelData<Y<>>& tireFy);
    WheelData<X<>> getVehicleFxFromTireForces(const WheelData<X<>>& tireFx,
                                                     const WheelData<Y<>>& tireFy);
    VehicleState springing(WheelData<float> loads);

   public:
    VehicleState* getState();
    Vehicle(const VehicleConfig& vehicleConfig, const TireConfig& tireConfig);
    Vehicle() = default;
    std::vector<std::array<float, 4>> getYawMomentDiagramPoints(
        const EnvironmentConfig& environmentConfig, float maxSteeringAngle = 10,
        float steeringAngleStep = 1, float maxSlipAngle = 10, float slipAngleStep = 1,
        float tolerance = 0.01, int maxIterations = 100);
};
