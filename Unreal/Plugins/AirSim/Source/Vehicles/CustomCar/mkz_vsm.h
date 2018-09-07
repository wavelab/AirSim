#ifndef MKZ_VSM_H
#define MKZ_VSM_H

#include "cMoose.h"

#define RAD2DEG 57.2958

// These consts need to be taken from the generated code and may change if the
// model is changed
const int InputSize =  NINP;
const int OutputSize = NOUT;
// workspace = 7 + 2 * NEQ + NPAR + NDFA + NEVT
const int FloatWorkspaceSize = 7+2*NEQ+NPAR+NDFA+NEVT;

struct Vector3 {
    float x, y, z;
};

struct ValidRange {
    double min;
    double max;
};

struct VehicleInput {
    double throttle_percent;
    double brake_position;
    double steering_angle;
};

struct WheelState {
    Vector3 position;
    Vector3 orientation;
    double angular_velocity;
};

struct VehicleState {
    double time;
    Vector3 position;
    Vector3 orientation;
    Vector3 velocity;
    Vector3 angular_velocity;

    WheelState fl_wheel_state;
    WheelState fr_wheel_state;
    WheelState rl_wheel_state;
    WheelState rr_wheel_state;
};

class MkzVsm {
    public:
        MkzVsm();

        void init();

        void init(Vector3 init_position);

        void setVehicleInput(VehicleInput input);

        double clampRange(ValidRange range, double input);

        int performSimulationStep();

        VehicleState getVehicleState();

    private:
        SolverStruct solver;
        double float_workspace[FloatWorkspaceSize];
        char error_buffer[1000];
        double input_array[InputSize];
        double output_array[OutputSize];
        double time;

        // Parameters
        double dt;
        int dt_multiplier;
        double steering_ratio;
        ValidRange steering_range;
        ValidRange throttle_range;
        ValidRange brake_range;
        Vector3 init_unreal_position;
};

#endif  // MKZ_VSM_H
