#pragma once

#include <array>
#include <memory>
#include <optional>
#include <vector>

#include "config/config.h"
#include "vehicle/aero/aero.h"
#include "coordTypes.h"
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

    WheelData<float> toeAngle;
    float ackermannPercentage;

    VehicleState state;

    Positioned<Aero> aero;

    WheelData<Positioned<std::unique_ptr<Tire>>> tires; 
    
    std::array<float, 2> getLatAccAndYawMoment(float tolerance, int maxIterations, 
                                               const EnvironmentConfig& environmentConfig);
    WheelData<Alpha<ISO8855>> calculateSlipAngles(float r, Vec3<float> velocity);
    WheelData<float> calculateSteeringAngles();
    WheelData<float> staticLoad(float earthAcc);
    Y<ISO8855> calculateLatAcc(const WheelData<X<ISO8855>>& tireForcesX,
                               const WheelData<Y<ISO8855>>& tireForcesY);
    WheelData<float> distributeForces(float totalForce, float frontDist, float leftDist);
    WheelData<float> totalTireLoads(float latAcc, const EnvironmentConfig& environmentConfig);
    WheelData<float> aeroLoad(const EnvironmentConfig& environmentConfig);
    WheelData<float> loadTransfer(float latAcc);
    WheelData<Y<ISO8855>> getVehicleFyFromTireForces(const WheelData<X<ISO8855>>& tireFx,
                                                     const WheelData<Y<ISO8855>>& tireFy);
    WheelData<X<ISO8855>> getVehicleFxFromTireForces(const WheelData<X<ISO8855>>& tireFx,
                                                     const WheelData<Y<ISO8855>>& tireFy);
    VehicleState springing(WheelData<float> loads);   
   public:
    VehicleState* getState();
    Vehicle(const VehicleConfig& vehicleConfig, const TireConfig& tireConfig);
    Vehicle() = default;
    std::vector<std::array<float, 4>> getYawMomentDiagramPoints(
        const EnvironmentConfig& environmentConfig, float maxSteeringAngle = 10, float steeringAngleStep = 1, float maxSlipAngle = 10, float slipAngleStep = 1, float tolerance = 0.01, int maxIterations = 100);
};
