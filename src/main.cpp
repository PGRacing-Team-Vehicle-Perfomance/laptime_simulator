#include <cstdio>
#include <iostream>

#include "config/config.h"
#include "simulation/skidpad.h"
#include "vehicle/vehicle.h"

int main() {
    VehicleConfig vc;
    EnvironmentConfig config;

    Vehicle v(vc);
    v.calculateYawMomentDiagram(100, 1, config);
}
