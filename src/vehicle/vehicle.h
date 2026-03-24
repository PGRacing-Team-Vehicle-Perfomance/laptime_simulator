#pragma once

#include <array>
#include <memory>
#include <optional>
#include <type_traits>
#include <vector>

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/aero/aero.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"

template <typename Frame>
class Vehicle {
    Mass<Frame> combinedTotalMass;

    Mass<Frame> combinedNonSuspendedMass;
    Mass<Frame> combinedSuspendedMass;

    WheelData<float> nonSuspendedMassAtWheels;
    WheelData<float> suspendedMassAtWheels;

    float rollCenterHeightFront;
    float rollCenterHeightBack;

    float antiRollStiffnessFront;
    float antiRollStiffnessRear;

    float frontTrackWidth;
    float rearTrackWidth;
    float trackDistance;

    WheelData<Alpha<Frame>> toeAngle;

    VehicleState<Frame> state;

    Positioned<Aero<Frame>, Frame> aero;

    WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>> tires;

    std::array<float, 2> getLatAccAndYawMoment(float tolerance, int maxIterations,
                                               const EnvironmentConfig<Frame>& environmentConfig);
    WheelData<Alpha<Frame>> calculateSlipAngles();
    void setSteering(Alpha<Frame> steeringAngle);
    WheelData<float> staticLoad(float earthAcc);
    Y<Frame> calculateLatAcc(const WheelData<X<Frame>>& tireForcesX,
                             const WheelData<Y<Frame>>& tireForcesY);
    WheelData<float> distributeForces(float totalForce, float frontDist, float leftDist);
    WheelData<float> totalTireLoads(Y<Frame> latAcc,
                                    const EnvironmentConfig<Frame>& environmentConfig);
    WheelData<float> aeroLoad(const EnvironmentConfig<Frame>& environmentConfig);
    WheelData<float> loadTransfer(Y<Frame> latAcc);
    WheelData<Y<Frame>> getVehicleFyFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                   const WheelData<Y<Frame>>& tireFy);
    WheelData<X<Frame>> getVehicleFxFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                   const WheelData<Y<Frame>>& tireFy);
    VehicleState<Frame> springing(WheelData<float> loads);
   public:
    Vehicle(const VehicleConfig<Frame>& vehicleConfig, WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>>&& tires);
    Vehicle() = default;

    std::vector<std::array<float, 4>> getYawMomentDiagramPoints(
        float speed, const EnvironmentConfig<Frame>& environmentConfig, float maxSteeringAngle = 10,
        float steeringAngleStep = 1, float maxSlipAngle = 10, float slipAngleStep = 1,
        float tolerance = 0.01, int maxIterations = 100);
};

#include "vehicle.inl"
