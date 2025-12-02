#include "vehicle/vehicle.h"
#include "simulation/skidpad.h"
#include "config/config.h"

#include <cstdio>
#include <iostream>

int main(){
    VehicleConfig vc;
    SimConfig sc;
    SkidPadConfig skidPadConfig;
    SimulationConstants simulationConstants;
    OptimizationConfig optconf;
    
    Vehicle v(vc);
    SkidPad s(v, sc, skidPadConfig, optconf, simulationConstants);
    s.run();
}