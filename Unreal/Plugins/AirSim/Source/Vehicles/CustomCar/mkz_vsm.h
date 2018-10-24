#ifndef MKZ_VSM_H
#define MKZ_VSM_H

#include "cMoose.h"

// These consts need to be taken from the generated code and may change if the
// model is changed
constexpr int InputSize =  NINP;
constexpr int OutputSize = NOUT;
// workspace = 7 + 2 * NEQ + NPAR + NDFA + NEVT
constexpr int FloatWorkspaceSize = 7+2*NEQ+NPAR+NDFA+NEVT;

const double wheel_radius = 0.3675;
const double wheelbase = 2.994;

struct Vector3 {
    float x, y, z;
};
// Column major [row column]
struct Matrix3x3 {
    float r1c1, r2c1, r3c1, r1c2, r2c2, r3c2, r1c3, r2c3, r3c3;
};

struct ValidRange {
    double min;
    double max;
};

struct VehicleInput {
    // Throttle is from 0.0 to 1.0
    double throttle_percent;
    // Brake is from 0.0 to 0.5, this represents the pedal position in the car
    double brake_position;
    // Steering angle is in radians and represents the steering wheel angle
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
    Matrix3x3 rotation;
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
