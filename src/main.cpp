#include <cstdio>
#include <iostream>

#include "config/config.h"
#include "simulation/simulation.h"

int main(int argc, char* argv[]) {
    std::string configPath;
    if (argc > 1) {
        configPath = argv[1];
    } else {
        std::cerr << "Usage: " << argv[0] << " <config_path>" << std::endl;
        return 1;
    }
    
    Config cfg(configPath);
    Simulation sim(cfg);
    
    std::vector<std::array<float, 4>> points = sim.run();

    FILE* f = fopen("build/yaw_diagram.csv", "w");
    if (f) {
        fprintf(f, "steering,slip,latAcc,yawMoment\n");
        for (const auto& p : points) {
            fprintf(f, "%f,%f,%f,%f\n", p[0], p[1], p[2], p[3]);
        }
        fclose(f);
        std::cout << "Wrote build/yaw_diagram.csv\n";
    } else {
        std::cerr << "Failed to open build/yaw_diagram.csv for writing\n";
    }
}
