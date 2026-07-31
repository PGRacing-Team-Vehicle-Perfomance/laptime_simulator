#pragma once

#include <array>
#include <memory>
#include <optional>
#include <type_traits>
#include <vector>

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/aero/aero.h"
#include "vehicle/steering/steeringTable.h"
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
    WheelData<Gamma<Frame>> camber;

    float driveBiasFront = 0;
    float brakeBiasFront = 0;
    float frontDiffLocking = 1;
    float rearDiffLocking = 1;
    float dragCoefficientArea = 0;
    float airDensityValue = 0;
    bool longEquilibriumEnabled = false;
    float targetLongAcc = 0;
    float longForceDemand = 0;

    float lastLatAcc = 0;
    float lastDemand = 0;
    float lastLongAcc = 0;
    float longitudinalAccEstimate = 0;
    WheelData<float> lastKappa{};

    VehicleState<Frame> state;

    Positioned<std::unique_ptr<AeroBase<Frame>>, Frame> aero;

    std::unique_ptr<SteeringTableBase<Frame>> steeringTable;

    WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>> tires;

    struct SolverStep {
        Y<Frame> latAcc;
        WheelData<X<Frame>> tireForcesX;
        WheelData<Y<Frame>> tireForcesY;
        WheelData<Z<Frame>> tireMomentsZ;
    };

    WheelData<Alpha<Frame>> calculateSlipAngles();
    WheelData<float> staticLoad(float earthAcc);
    Y<Frame> calculateLatAcc(const WheelData<X<Frame>>& tireForcesX,
                             const WheelData<Y<Frame>>& tireForcesY);
    X<Frame> calculateLongAcc(const WheelData<X<Frame>>& tireForcesX,
                              const WheelData<Y<Frame>>& tireForcesY);
    X<Frame> calculateBodyLongAcc(const WheelData<X<Frame>>& tireForcesX,
                                  const WheelData<Y<Frame>>& tireForcesY);
    float slipRatioForForce(size_t wheel, float load, Alpha<Frame> slipAngle, Gamma<Frame> camber,
                            float targetFx);
    float wheelLongSpeed(size_t wheel);
    void solveAxle(size_t leftWheel, size_t rightWheel, const WheelData<float>& loads,
                   const WheelData<Alpha<Frame>>& slipAngles, float axleDemand, bool hasDiff,
                   float locking, float& leftSlipRatio, float& rightSlipRatio);
    SolverStep solveLatAcc(const Config& config, float tolerance, int maxIterations);
    SolverStep solveCoupled(const Config& config, float tolerance, int maxIterations);
    void computeTireForces(const WheelData<float>& loads, const WheelData<Alpha<Frame>>& slipAngles,
                           SolverStep& step);
    WheelData<float> distributeForces(float totalForce, float frontDist, float leftDist);
    WheelData<float> totalTireLoads(Y<Frame> latAcc, const Config& config);
    WheelData<float> aeroLoad(const Config& config);
    WheelData<float> loadTransfer(Y<Frame> latAcc);
    WheelData<float> loadTransferLongitudinal(float longAcc);
    void resolveAxleLiftOff(float& leftLoad, float& rightLoad);
    WheelData<Y<Frame>> getVehicleFyFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                   const WheelData<Y<Frame>>& tireFy);
    WheelData<X<Frame>> getVehicleFxFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                   const WheelData<Y<Frame>>& tireFy);
    WheelData<Y<Frame>> getVelocityFyFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                    const WheelData<Y<Frame>>& tireFy);
    SolverStep evaluateAt(Y<Frame> testLatAcc, const Config& config);

   public:
    Vehicle(const Config& config,
            WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>>&& tires,
            Positioned<std::unique_ptr<AeroBase<Frame>>, Frame>&& aero,
            std::unique_ptr<SteeringTableBase<Frame>>&& steeringTable);

    struct Continuity {
        float latAcc;
        float demand;
        float longAcc;
        WheelData<float> kappa;
    };

    void setChassisSlipAngle(Alpha<Frame> chassisSlipAngle);
    void setSteeringAngle(Alpha<Frame> steeringAngle);
    void setSpeed(float speed);
    void resetContinuity();
    Continuity continuity() const;
    void continuity(const Continuity& snapshot);

    std::array<float, 2> calculateLatAccAndYawMoment(float tolerance, int maxIterations,
                                                     const Config& config);
};

#include "vehicle.inl"
