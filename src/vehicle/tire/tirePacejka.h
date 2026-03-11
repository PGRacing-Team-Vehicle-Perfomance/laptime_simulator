#pragma once

#include "config/config.h"
#include "coordTypes.h"
#include "types.h"
#include "vehicle/tire/tire.h"

enum Side { Left, Right };

class PacejkaModel {
   protected:
    Side sideRelativeToVehicle;

    float PCY1;
    float PDY1;
    float PDY2;
    float PDY3;
    float PEY1;
    float PEY2;
    float PEY3;
    float PEY4;
    float PKY1;
    float PKY2;
    float PKY3;
    float PHY1;
    float PHY2;
    float PHY3;
    float PVY1;
    float PVY2;
    float PVY3;
    float PVY4;
    float FNOMIN;

    float computeFy(float verticalLoad, Alpha<SAE> slipAngle, float slipRatio);

   public:
    PacejkaModel() = default;
    PacejkaModel(const TireConfig& config, Side sideRelativeToVehicle);
};

template <typename External = ISO8855>
class TirePacejka : public Tire<SAE, External>, private PacejkaModel {
   public:
    TirePacejka(const TireConfig& config, bool isDriven, Side side)
        : Tire<SAE, External>(isDriven), PacejkaModel(config, side) {}

    void calculateInternal(float load, Alpha<SAE> slip, float slipRatio) override {
        float FySAE = computeFy(load, slip, slipRatio);
        this->internalForce = Force<SAE>(Vec<SAE>(0, FySAE, 0), Vec<SAE>(0, 0, 0));
        this->internalTorque = Torque<SAE>(0, 0, 0);
    }
};
