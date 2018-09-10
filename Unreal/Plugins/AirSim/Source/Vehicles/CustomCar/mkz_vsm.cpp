#include "mkz_vsm.h"
#include "stdio.h"

MkzVsm::MkzVsm() {
    solver.w = float_workspace;
    solver.err = 0;
    // error_buffer[0] = NULL;

    // Param defaults
    this->steering_ratio = 14.8;
    this->steering_range.min = -8.48;
    this->steering_range.max = 8.48;
    this->throttle_range.min = 0.0;
    this->throttle_range.max = 1.0;
    this->brake_range.min = 0.0;
    this->brake_range.max = 0.5;
}

void MkzVsm::init() {
    // Initialize the input array, assume slope never changes
    for (int i = 0; i < InputSize; ++i) {
        input_array[i] = 0;
    }
    input_array[2] = 0;

    this->time = 0.0;
    double t0 = 0.0;
    this->dt = 0.002;
    this->dt_multiplier = 10;
    double *ic = NULL;
    double *p = NULL;

    SolverSetup(t0, ic, input_array, p, output_array, dt, &solver);

    if (solver.err > 0) {
        printf("%s\n", solver.buf);
    }
    solver.buf = error_buffer;
}

void MkzVsm::init(Vector3 init_position) {
    init();
    init_unreal_position = init_position;
}

void MkzVsm::setVehicleInput(VehicleInput input) {
    // Clamp to the limits of acceptable input
    VehicleInput clamped_input;
    clamped_input.throttle_percent =
        clampRange(throttle_range, input.throttle_percent);
    clamped_input.brake_position =
        clampRange(brake_range, input.brake_position);
    clamped_input.steering_angle =
        clampRange(steering_range, input.steering_angle);

    // Model expects percent to range from 0.0 to 100.0
    this->input_array[0] = clamped_input.throttle_percent * 100.0;
    this->input_array[1] = clamped_input.brake_position;
    // Convert steering angle to tire angle
    this->input_array[2] = clamped_input.steering_angle / this->steering_ratio;
}

double MkzVsm::clampRange(ValidRange range, double input) {
    if (input > range.max) {
        return range.max;
    }
    if (input < range.min) {
        return range.min;
    }
    return input;
}

int MkzVsm::performSimulationStep() {
    for(int i = 0; i < this->dt_multiplier; i++) {
        RK4Step(input_array, &solver);
        SolverOutputs(output_array, &solver);

        if (solver.err > 0) {
            printf("%s\n", solver.buf);
            return 1;
        }
        this->time += this->dt;
    }
    return 0;
}

VehicleState MkzVsm::getVehicleState() {
    Vector3 position;
    Vector3 orientation;
    Vector3 velocity;
    Vector3 angular_velocity;
    VehicleState vehicle_state;
    WheelState fl_ws;
    WheelState fr_ws;
    WheelState rl_ws;
    WheelState rr_ws;

    // Vehicle body
    position.x = output_array[9]; 
    position.y = -output_array[10];
    position.z = output_array[11];
    vehicle_state.position = position;
    orientation.x = output_array[3]; 
    orientation.y = output_array[4];
    orientation.z = output_array[5];
    vehicle_state.orientation = orientation;
    velocity.x = output_array[12];
    velocity.y = output_array[13];
    velocity.z = output_array[14];
    vehicle_state.velocity = velocity;
    angular_velocity.x = output_array[31];
    angular_velocity.y = output_array[32];
    angular_velocity.z = output_array[33];
    vehicle_state.angular_velocity = angular_velocity;

    // Wheel states
    fl_ws.angular_velocity = output_array[18];
    vehicle_state.fl_wheel_state = fl_ws;
    fr_ws.angular_velocity = output_array[23];
    vehicle_state.fr_wheel_state = fr_ws;
    rl_ws.angular_velocity = output_array[46];
    vehicle_state.rl_wheel_state = rl_ws;
    rr_ws.angular_velocity = output_array[53];
    vehicle_state.rr_wheel_state = rr_ws;

    vehicle_state.time = this->time;

    return (vehicle_state);
}
