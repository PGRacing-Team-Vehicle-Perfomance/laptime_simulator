#include <cstdio>
#include <iostream>

#include "config/config.h"
#include "simulation/diagramMetrics.h"
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

    if (argc > 2 && std::string(argv[2]) == "tire") {
        sim.dumpTireModel("build/tire_model.csv");
        return 0;
    }

    std::vector<DiagramSample> points = sim.run();

    FILE* f = fopen("build/yaw_diagram.csv", "w");
    if (f) {
        writeDiagramHeader(f);
        for (const DiagramSample& p : points) {
            writeDiagramRow(f, p);
        }
        fclose(f);
        std::cout << "Wrote build/yaw_diagram.csv\n";
    } else {
        std::cerr << "Failed to open build/yaw_diagram.csv for writing\n";
    }

    std::vector<MetricPoint> metricPoints = finiteDiagramPoints(points);
    std::vector<MetricPoint> hull = convexHull(metricPoints);
    DiagramMetrics metrics = computeDiagramMetrics(metricPoints, hull);

    FILE* mf = fopen("build/metrics.csv", "w");
    if (mf) {
        writeMetricsCsv(mf, metrics);
        fclose(mf);
        std::cout << "Wrote build/metrics.csv\n";
    } else {
        std::cerr << "Failed to open build/metrics.csv for writing\n";
    }

    FILE* hf = fopen("build/hull.csv", "w");
    if (hf) {
        writeHullCsv(hf, hull);
        fclose(hf);
        std::cout << "Wrote build/hull.csv\n";
    } else {
        std::cerr << "Failed to open build/hull.csv for writing\n";
    }
}
