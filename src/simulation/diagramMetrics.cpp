#include "simulation/diagramMetrics.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

namespace {

// axis: 0 = latAcc, 1 = yawMoment
double coord(const MetricPoint& p, int axis) { return axis == 0 ? p.latAcc : p.yawMoment; }

double crossProduct(const MetricPoint& origin, const MetricPoint& a, const MetricPoint& b) {
    return (a.latAcc - origin.latAcc) * (b.yawMoment - origin.yawMoment) -
           (a.yawMoment - origin.yawMoment) * (b.latAcc - origin.latAcc);
}

std::vector<double> hullCrossingsAt(const std::vector<MetricPoint>& hull, int fixedAxis,
                                    double fixedValue) {
    int freeAxis = 1 - fixedAxis;
    std::vector<double> crossings;
    size_t vertexCount = hull.size();
    for (size_t i = 0; i < vertexCount; i++) {
        const MetricPoint& start = hull[i];
        const MetricPoint& end = hull[(i + 1) % vertexCount];
        double a = coord(start, fixedAxis);
        double b = coord(end, fixedAxis);
        if (a == b) continue;
        if ((a - fixedValue) * (b - fixedValue) <= 0.0) {
            double fraction = (fixedValue - a) / (b - a);
            crossings.push_back(coord(start, freeAxis) +
                                fraction * (coord(end, freeAxis) - coord(start, freeAxis)));
        }
    }
    return crossings;
}

}  // namespace

std::vector<MetricPoint> finiteDiagramPoints(const std::vector<DiagramSample>& samples) {
    std::vector<MetricPoint> points;
    points.reserve(samples.size());
    for (const DiagramSample& sample : samples) {
        double latAcc = sample.solution.latAcc;
        double yawMoment = sample.solution.yawMoment;
        if (std::isfinite(latAcc) && std::isfinite(yawMoment)) {
            points.push_back({latAcc, yawMoment});
        }
    }
    return points;
}

std::vector<MetricPoint> convexHull(std::vector<MetricPoint> points) {
    // dedup and sort (latAcc, yawMoment)
    std::set<std::pair<double, double>> unique;
    for (const MetricPoint& p : points) unique.insert({p.latAcc, p.yawMoment});
    std::vector<MetricPoint> sorted;
    sorted.reserve(unique.size());
    for (const auto& [latAcc, yawMoment] : unique) sorted.push_back({latAcc, yawMoment});
    if (sorted.size() <= 2) return sorted;

    std::vector<MetricPoint> lower;
    for (const MetricPoint& p : sorted) {
        while (lower.size() >= 2 && crossProduct(lower[lower.size() - 2], lower.back(), p) <= 0.0) {
            lower.pop_back();
        }
        lower.push_back(p);
    }
    std::vector<MetricPoint> upper;
    for (auto it = sorted.rbegin(); it != sorted.rend(); ++it) {
        while (upper.size() >= 2 &&
               crossProduct(upper[upper.size() - 2], upper.back(), *it) <= 0.0) {
            upper.pop_back();
        }
        upper.push_back(*it);
    }
    std::vector<MetricPoint> hull;
    hull.reserve(lower.size() + upper.size());
    hull.insert(hull.end(), lower.begin(), lower.end() - 1);
    hull.insert(hull.end(), upper.begin(), upper.end() - 1);
    return hull;
}

DiagramMetrics computeDiagramMetrics(const std::vector<MetricPoint>& points,
                                     const std::vector<MetricPoint>& hull) {
    DiagramMetrics metrics;
    if (points.empty()) return metrics;

    // first maximal/minimal in diagram order
    const MetricPoint* maxLatacc = &points[0];
    const MetricPoint* minLatacc = &points[0];
    const MetricPoint* maxMoment = &points[0];
    const MetricPoint* minMoment = &points[0];
    for (const MetricPoint& p : points) {
        if (p.latAcc > maxLatacc->latAcc) maxLatacc = &p;
        if (p.latAcc < minLatacc->latAcc) minLatacc = &p;
        if (p.yawMoment > maxMoment->yawMoment) maxMoment = &p;
        if (p.yawMoment < minMoment->yawMoment) minMoment = &p;
    }
    metrics.maxLataccOverall = *maxLatacc;
    metrics.minLataccOverall = *minLatacc;
    metrics.maxMomentOverall = *maxMoment;
    metrics.minMomentOverall = *minMoment;

    std::vector<double> lataccAtZeroMoment = hullCrossingsAt(hull, /*fixedAxis=*/1, 0.0);
    if (!lataccAtZeroMoment.empty()) {
        auto [mn, mx] = std::minmax_element(lataccAtZeroMoment.begin(), lataccAtZeroMoment.end());
        metrics.maxLataccAtZeroMoment = MetricPoint{*mx, 0.0};
        metrics.minLataccAtZeroMoment = MetricPoint{*mn, 0.0};
    }
    std::vector<double> momentAtZeroLatacc = hullCrossingsAt(hull, /*fixedAxis=*/0, 0.0);
    if (!momentAtZeroLatacc.empty()) {
        auto [mn, mx] = std::minmax_element(momentAtZeroLatacc.begin(), momentAtZeroLatacc.end());
        metrics.maxMomentAtZeroLatacc = MetricPoint{0.0, *mx};
        metrics.minMomentAtZeroLatacc = MetricPoint{0.0, *mn};
    }
    return metrics;
}

void writeMetricsCsv(FILE* f, const DiagramMetrics& metrics) {
    fprintf(f, "metric,latAcc,yawMoment,description\n");
    struct Row {
        const char* key;
        const std::optional<MetricPoint>& point;
        const char* description;
    };
    const Row rows[] = {
        {"max_latacc_at_zero_moment", metrics.maxLataccAtZeroMoment,
         "max lateral acc at yaw moment 0 (trimmed grip limit, +)"},
        {"min_latacc_at_zero_moment", metrics.minLataccAtZeroMoment,
         "min lateral acc at yaw moment 0 (trimmed grip limit, -)"},
        {"max_latacc_overall", metrics.maxLataccOverall,
         "peak lateral acc and the yaw moment where it occurs (+)"},
        {"min_latacc_overall", metrics.minLataccOverall,
         "peak lateral acc and the yaw moment where it occurs (-)"},
        {"max_moment_at_zero_latacc", metrics.maxMomentAtZeroLatacc,
         "max yaw moment at lateral acc 0 (rotation authority, +)"},
        {"min_moment_at_zero_latacc", metrics.minMomentAtZeroLatacc,
         "min yaw moment at lateral acc 0 (rotation authority, -)"},
        {"max_moment_overall", metrics.maxMomentOverall,
         "peak yaw moment and the lateral acc where it occurs (+)"},
        {"min_moment_overall", metrics.minMomentOverall,
         "peak yaw moment and the lateral acc where it occurs (-)"},
    };
    for (const Row& row : rows) {
        if (!row.point.has_value()) continue;
        fprintf(f, "%s,%.4f,%.4f,\"%s\"\n", row.key, row.point->latAcc, row.point->yawMoment,
                row.description);
    }
}

void writeHullCsv(FILE* f, const std::vector<MetricPoint>& hull) {
    fprintf(f, "latAcc,yawMoment\n");
    for (const MetricPoint& p : hull) {
        fprintf(f, "%.6g,%.6g\n", p.latAcc, p.yawMoment);
    }
}
