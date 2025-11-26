#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <array>

struct SimConfig{
    float errDelta = 0.001;
    float startSpeed = 9.0;
    float dragDt = 0.002; // Optimized based on test results
    float earthAcc = 9.81;
    float airDensity = 1.25;
    int maxIterConv = 10;
};

struct VehicleConfig{
    float mass = 280.0;
    float frontTrackWidth = 1.256;
    float rearTrackWidth = 1.216;
    float cogHeight = 0.33;
    float frontWeightDist = 0.45;
    float leftWeightDist = 0.5;
    float cla = 4.3;
    float leftAeroDist = 0.5;
    float frontAeroDist = 0.5;
    float quadFac = -0.0002;
    float linFac = 1.8109;
    float cdA = 1.3;
    float cRr = 0.015;
    float wheelbase = 1.53;
    float wheelRadius = 0.2;
    std::unordered_map<int, float> torqueCurve = {
            {3000, 20.0},
            {3500, 56.0},
            {4000, 62.5},
            {4500, 62.5},
            {5000, 66.0},
            {5500, 67.0},
            {6000, 64.0},
            {6500, 64.0},
            {7000, 66.0},
            {7500, 65.0},
            {8000, 66.0},
            {8500, 65.0},
            {9000, 63.0},
            {9500, 63.0},
            {10000, 62.5},
            {10500, 60.0},
            {11000, 58.0},
            {11500, 56.0},
            {12000, 50.0}
    };

    std::vector<float> gearRatios = {2.615, 1.857, 1.565, 1.35, 1.238, 1.136};
    float finalDriveRatio = 6.048;
    float shiftTime = 0.05;
    int redlineRpm = 12000;
    int idleRpm = 3000;
    int maxTorqueRpm = 5500;
    std::vector<std::string> drivenWheels = {"LR", "RR"};
};
    

struct SkidPadConfig {
    float minDiameter = 15.25;
    float maxDiameter = 21.25;
    float tireScalingFactor = 0.77;
    float historicalTime = 5.1;
    float historicalBestTime = 4.856;
    float historicalPMax = 75;
    float diameterOffset = 1.3;
};
   
struct SimulationConstants {
    int trackMaxIterations = 50; // Reduced for early stopping
    int trackMinIterations = 5; // Ensure stability
    float dragTimeout = 20.0;
};


struct OptimizationConfig {
    /*
    plot_dir: str = 'plots'
    max_workers: int = max(1, cpu_count() - 1)  # Optimized for more cores
    single_sweep_range: Tuple[float, float] = (0.9, 1.1)
    single_sweep_points: int = 30
    multi_sweep_range: Tuple[float, float, int] = (0.85, 1.15, 5)  # Reduced points for coarse grid
    max_combinations: int = 100000  # Limit combinations
    exclude_params: Set[str] = field(
        default_factory=lambda: {
            #'quad_fac',
            #'lin_fac',
            'left_weight_dist',
            'left_aero_dist',
            #'c_rr',
            #'wheelbase',
            'wheel_radius',
            'final_drive_ratio',
            'shift_time',
            'redline_rpm',
            'idle_rpm',
            'max_torque_rpm',
            'torque_curve',
            'gear_ratios',
            'driven_wheels',
        }
    )
    */
    std::array<float, 3> pointsCoefficients = {0.95, 0.5625, 0.05};
    bool enablePlots = false;  // Disable plotting by default
};
