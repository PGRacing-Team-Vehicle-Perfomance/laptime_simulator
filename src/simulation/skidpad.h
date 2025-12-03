#include "vehicle/vehicle.h"

class SkidPad
{
    float diameter;
    float radius;
    float trackLength;
    SkidPadConfig trackConfig;
    SimulationConstants simulationConstants;
    SimConfig simConfig;
    Vehicle& vehicle;
    OptimizationConfig optimizationConfig;

public:
    SkidPad(Vehicle &vehicle, SimConfig simConfig, SkidPadConfig skidPadConfig, OptimizationConfig optimizationConfig, SimulationConstants simulationConstants);
    void run();
    float calculatePoints(float time, float tMax, float pMax);
};
