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

    WheelData<Alpha<Frame>> calculateSlipAngles();
    WheelData<float> staticLoad(float earthAcc);
    Y<Frame> calculateLatAcc(const WheelData<X<Frame>>& tireForcesX,
                             const WheelData<Y<Frame>>& tireForcesY);
    WheelData<float> distributeForces(float totalForce, float frontDist, float leftDist);
    WheelData<float> totalTireLoads(Y<Frame> latAcc,
                                    const Config& config);
    WheelData<float> aeroLoad(const Config& config);
    WheelData<float> loadTransfer(Y<Frame> latAcc);
    WheelData<Y<Frame>> getVehicleFyFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                   const WheelData<Y<Frame>>& tireFy);
    WheelData<X<Frame>> getVehicleFxFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                   const WheelData<Y<Frame>>& tireFy);
    WheelData<Y<Frame>> getVelocityFyFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                           const WheelData<Y<Frame>>& tireFy);

   public:
    Vehicle(const Config& config,
            WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>>&& tires);
    Vehicle() = default;

    void setChassisSlipAngle(Alpha<Frame> chassisSlipAngle);
    void setSteeringAngle(Alpha<Frame> steeringAngle);
    void setSpeed(float speed);

    std::array<float, 2> calculateLatAccAndYawMoment(float tolerance, int maxIterations,
                                               const Config& config);
};

#include "vehicle.inl"
