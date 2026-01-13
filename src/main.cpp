#include <cstdio>
#include <iostream>

#include "config/config.h"
#include "simulation/skidpad.h"
#include "vehicle/vehicle.h"

int main() {
    VehicleConfig vc;
    EnvironmentConfig config;

    Vehicle v(vc);
    v.getState()->velocity->amplitude = 100;
    v.calculateYawMomentDiagram(1, config);
}
