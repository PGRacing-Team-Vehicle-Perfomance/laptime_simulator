#include "simulation/acceleration.h"

#include <cmath>

Acceleration::Acceleration(Vehicle &vehicle, SimConfig simConfig, SimulationConstants simulationConstants, AccelerationConfig dragConfig) : dragConfig(dragConfig), Simulation(vehicle, simConfig, simulationConstants)
{
    currentGear = 0;
    isShifting = false;
    shiftEndTime = 0.0;
}

float Acceleration::run()
{
    float pos = 0, vel = dragConfig.startSpeed, time = 0, prevAcc = 0.0;
    float mass = vehicle.getMass(), c_rr = vehicle.getCRR(), cd_a = vehicle.getCDA();
    float tolerance = simConfig.errDelta;      // Reuse err_delta for consistency
    int maxIterConv = simConfig.maxIterConv; // Max iterations per time step for convergence
    float acc = 0;

    try
    {
        while (pos < dragConfig.length)
        {
            float currentTime = time + simConfig.dragDt;

            if (isShifting)
            {
                acc = 0.0;
                if (currentTime >= shiftEndTime)
                {
                    isShifting = false;
                }
            }
            else
            {
                float rpm = vehicle.speedToRpm(vel, currentGear);
                if (rpm >= vehicle.getMaxTorqueRpm() && currentGear < vehicle.getGearCount() - 1)
                {
                    currentGear += 1;
                    isShifting = true;
                    shiftEndTime = currentTime + vehicle.getShiftTime();
                    continue;
                }

                // Converged calculation: Fixed-point iteration for acc
                float rrForce = mass * simConfig.earthAcc * c_rr;
                float dragForce = 0.5 * cd_a * simConfig.airDensity * std::pow(vel, 2);
                float powerThrust = vehicle.getPowerThrust(vel, currentGear);

                // Initial guess for acc (previous for stability)
                float accGuess = prevAcc;
                for (int i = 0; i < maxIterConv; i++)
                {
                    // Compute traction using current guess for load transfer
                    float tractionMax = vehicle.getTireForces(vel, accGuess, simConfig, false);
                    float thrust = std::min(powerThrust, tractionMax);
                    float accNew = (thrust - dragForce - rrForce) / mass;

                    // Check convergence
                    if (std::abs(accNew - accGuess) < tolerance)
                    {
                        break;
                    }
                    accGuess = accNew; // Relax to new value (or use acc_guess = 0.5 * acc_guess + 0.5 * acc_new for damping)
                }
                acc = accGuess; // Use converged value
            }
            vel += acc * simConfig.dragDt;
            pos += vel * simConfig.dragDt;
            time = currentTime;
            prevAcc = acc; // Still update prev_acc for next step's initial guess
            if (time > simulationConstants.dragTimeout)
            {
                break;
            }
        }

        return time;
    }
    catch (int e)
    {
        return -1;
    }
}

float Acceleration::calculatePoints(float time, const PointsConfig &pointsConfig) const
{
    float a = pointsConfig.pointsCoefficients[0];
    float b = pointsConfig.pointsCoefficients[1];
    float c = pointsConfig.pointsCoefficients[2];
    float pMax = pointsConfig.historicalPMax;
    float tMax = pointsConfig.historicalBestTime * 1.5;
    return a * pMax * ((tMax / time) - 1) / 0.5 + c * pMax;
}
