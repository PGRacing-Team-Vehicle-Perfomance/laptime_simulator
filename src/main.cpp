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
    v.getState()->velocity.setLength(11);
    // v.calculateYawMomentDiagram(0.0001, config);

    // Generate dataset and write CSV for external visualization
    auto points = v.getYawMomentDiagramPoints(2, 100, config);
    // Ensure build directory exists at runtime when running from project root
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
