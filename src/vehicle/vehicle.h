#pragma once

#include <array>
#include <memory>
#include <optional>
#include <type_traits>
#include <vector>

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/aero/aero.h"
#include "vehicle/differential/differential.h"
#include "vehicle/steering/steeringTable.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"

struct PointSolution {
    float latAcc;
    float yawMoment;
    float longAcc;
    float aeroDownforce;
    float aeroDrag;
    float totalLoad;
    WheelData<float> load;
    WheelData<float> slipAngle;
    WheelData<float> slipRatio;
    WheelData<float> forceX;
    WheelData<float> forceY;
    WheelData<float> momentZ;
    WheelData<float> camber;
    WheelData<float> aeroLoad;
};

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
    float dragCoefficientArea = 0;
    float airDensityValue = 0;
    bool longEquilibriumEnabled = false;
    float targetLongAcc = 0;
    float longForceDemand = 0;
    float tireCalibrationSlip = 0;
    float lateralAccBracketG = 4;

    float longitudinalAccEstimate = 0;

    VehicleState<Frame> state;

    Positioned<std::unique_ptr<AeroBase<Frame>>, Frame> aero;

    std::unique_ptr<SteeringTableBase<Frame>> steeringTable;

    WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>> tires;

    std::unique_ptr<DifferentialBase<Frame>> differential;

    struct SolverStep {
        Y<Frame> latAcc;
        WheelData<X<Frame>> tireForcesX;
        WheelData<Y<Frame>> tireForcesY;
        WheelData<Z<Frame>> tireMomentsZ;
        WheelData<float> loads;
        WheelData<float> slipRatios;
    };

    WheelData<Alpha<Frame>> calculateSlipAngles();
    WheelData<float> staticLoad(float earthAcc);
    Y<Frame> calculateLatAcc(const WheelData<X<Frame>>& tireForcesX,
                             const WheelData<Y<Frame>>& tireForcesY);
    X<Frame> calculatePathLongAcc(const WheelData<X<Frame>>& tireForcesX,
                                  const WheelData<Y<Frame>>& tireForcesY);
    X<Frame> calculateBodyLongAcc(const WheelData<X<Frame>>& tireForcesX,
                                  const WheelData<Y<Frame>>& tireForcesY);
    AxleWheel<Frame> axleWheel(size_t wheel, const WheelData<float>& loads,
                               const WheelData<Alpha<Frame>>& slipAngles);
    void solveAxle(size_t leftWheel, size_t rightWheel, const WheelData<float>& loads,
                   const WheelData<Alpha<Frame>>& slipAngles, float axleDemand,
                   float& leftSlipRatio, float& rightSlipRatio);
    SolverStep bisectLatAcc(const Config& config, float maxLatAcc, float tolerance,
                            int maxIterations);
    SolverStep bisectDemand(const Config& config, float maxForce, float maxLatAcc, float tolerance,
                            int maxIterations);
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
    PointSolution invalidSolution();
    PointSolution assembleSolution(float latAcc, float yawMoment, const SolverStep& step,
                                   const WheelData<Alpha<Frame>>& slipAngles, const Config& config);

   public:
    Vehicle(const Config& config,
            WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>>&& tires,
            Positioned<std::unique_ptr<AeroBase<Frame>>, Frame>&& aero,
            std::unique_ptr<SteeringTableBase<Frame>>&& steeringTable,
            std::unique_ptr<DifferentialBase<Frame>>&& differential);

    void setChassisSlipAngle(Alpha<Frame> chassisSlipAngle);
    void setSteeringAngle(Alpha<Frame> steeringAngle);
    void setSpeed(float speed);

    PointSolution solveDiagramPoint(float tolerance, int maxIterations, const Config& config);
};

#include "vehicle.inl"
