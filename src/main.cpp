#include "config/config.h"
#include "vehicle/vehicle.h"
#include "simulation/skidpad.h"
#include "simulation/acceleration.h"

#include <cstdio>
#include <iostream>

int main()
{
    VehicleConfig vc;
    SimConfig sc;
    SkidPadConfig skidPadConfig;
    SimulationConstants simulationConstants;
    PointsConfig pointsConfig;

    Vehicle v(vc);
    
    SkidPad s(v, sc, simulationConstants, skidPadConfig);
    float lapTime = s.run();
    float points = s.calculatePoints(lapTime, pointsConfig);
    printf("lap time: %f\npoints: %f\n", lapTime, points);

    AccelerationConfig accelerationConfig;
    
    Acceleration a(v, sc, simulationConstants, accelerationConfig);
    lapTime = a.run();

    pointsConfig.historicalBestTime = 4.23;
    pointsConfig.historicalPMax = 73;
    points = a.calculatePoints(lapTime, pointsConfig);
    printf("lap time: %f\npoints: %f\n", lapTime, points);
}