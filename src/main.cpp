#include <cstdio>
#include <iostream>

#include "config/config.h"
#include "simulation/skidpad.h"
#include "vehicle/vehicle.h"

int main() {
    TireConfig tireConfig;
    VehicleConfig vehicleConfig;
    EnvironmentConfig environmentConfig;
    
    Vehicle v(vehicleConfig, tireConfig);
    v.getState()->velocity.setLength(11);
    
    auto points = v.getYawMomentDiagramPoints(environmentConfig, 25, 0.2, 20, 1);
    FILE *f = fopen("build/yaw_diagram.csv", "w");
    if (f) {
        fprintf(f, "steering,slip,latAcc,yawMoment\n");
        for (const auto &p : points) {
            fprintf(f, "%f,%f,%f,%f\n", p[0], p[1], p[2], p[3]);
        }
        fclose(f);
        std::cout << "Wrote build/yaw_diagram.csv\n";
    } else {
        std::cerr << "Failed to open build/yaw_diagram.csv for writing\n";
    }
}
