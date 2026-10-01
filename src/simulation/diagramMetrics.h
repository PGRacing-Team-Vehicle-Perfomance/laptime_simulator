#pragma once

#include <cstdio>
#include <optional>
#include <vector>

#include "simulation/simulation.h"

// MMM operating-point metrics over the (latAcc, yawMoment) diagram (see manifesto #42)
struct MetricPoint {
    double latAcc;
    double yawMoment;
};

struct DiagramMetrics {
    std::optional<MetricPoint> maxLataccAtZeroMoment;
    std::optional<MetricPoint> minLataccAtZeroMoment;
    std::optional<MetricPoint> maxLataccOverall;
    std::optional<MetricPoint> minLataccOverall;
    std::optional<MetricPoint> maxMomentAtZeroLatacc;
    std::optional<MetricPoint> minMomentAtZeroLatacc;
    std::optional<MetricPoint> maxMomentOverall;
    std::optional<MetricPoint> minMomentOverall;
};

// finite diagram points
std::vector<MetricPoint> finiteDiagramPoints(const std::vector<DiagramSample>& samples);

// convex hull (Andrew's monotone chain)
std::vector<MetricPoint> convexHull(std::vector<MetricPoint> points);

DiagramMetrics computeDiagramMetrics(const std::vector<MetricPoint>& points,
                                     const std::vector<MetricPoint>& hull);

void writeMetricsCsv(FILE* f, const DiagramMetrics& metrics);
void writeHullCsv(FILE* f, const std::vector<MetricPoint>& hull);
