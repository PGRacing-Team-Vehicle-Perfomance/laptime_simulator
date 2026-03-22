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

template <typename VFrame, typename TFrame>
class Vehicle {
    Mass<VFrame> combinedTotalMass;

    Mass<VFrame> combinedNonSuspendedMass;
    Mass<VFrame> combinedSuspendedMass;

    WheelData<float> nonSuspendedMassAtWheels;
    WheelData<float> suspendedMassAtWheels;

    float rollCenterHeightFront;
    float rollCenterHeightBack;

    float antiRollStiffnessFront;
    float antiRollStiffnessRear;

    float frontTrackWidth;
    float rearTrackWidth;
    float trackDistance;

    WheelData<Alpha<VFrame>> toeAngle;

    VehicleState<VFrame> state;

    Positioned<Aero<VFrame>, VFrame> aero;

    WheelData<Positioned<std::unique_ptr<Tire<TFrame>>, VFrame>> tires;

    std::array<float, 2> getLatAccAndYawMoment(float tolerance, int maxIterations,
                                               const EnvironmentConfig<VFrame>& environmentConfig);
    WheelData<Alpha<VFrame>> calculateSlipAngles();
    void setSteering(Alpha<VFrame> steeringAngle);
    WheelData<float> staticLoad(float earthAcc);
    Y<VFrame> calculateLatAcc(const WheelData<X<VFrame>>& tireForcesX,
                             const WheelData<Y<VFrame>>& tireForcesY);
    WheelData<float> distributeForces(float totalForce, float frontDist, float leftDist);
    WheelData<float> totalTireLoads(Y<VFrame> latAcc,
                                    const EnvironmentConfig<VFrame>& environmentConfig);
    WheelData<float> aeroLoad(const EnvironmentConfig<VFrame>& environmentConfig);
    WheelData<float> loadTransfer(Y<VFrame> latAcc);
    WheelData<Y<VFrame>> getVehicleFyFromTireForces(const WheelData<X<VFrame>>& tireFx,
                                                   const WheelData<Y<VFrame>>& tireFy);
    WheelData<X<VFrame>> getVehicleFxFromTireForces(const WheelData<X<VFrame>>& tireFx,
                                                   const WheelData<Y<VFrame>>& tireFy);
    VehicleState<VFrame> springing(WheelData<float> loads);
   public:
    Vehicle(const VehicleConfig<VFrame>& vehicleConfig, const TireConfig& tireConfig);
    Vehicle() = default;
    std::vector<std::array<float, 4>> getYawMomentDiagramPoints(
        float speed, const EnvironmentConfig<VFrame>& environmentConfig, float maxSteeringAngle = 10,
        float steeringAngleStep = 1, float maxSlipAngle = 10, float slipAngleStep = 1,
        float tolerance = 0.01, int maxIterations = 100);
};

#include "vehicle.inl"
