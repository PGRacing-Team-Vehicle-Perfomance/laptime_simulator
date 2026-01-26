#include <cstdio>
#include <iostream>

#include "config/config.h"
#include "simulation/skidpad.h"
#include "vehicle/vehicle.h"

int main() {
    TireConfig tc;
    VehicleConfig vc;
    EnvironmentConfig config;
    
    Vehicle v(vc, tc);
    v.getState()->velocity.setLength(100);
    v.calculateYawMomentDiagram(1, config);
}
