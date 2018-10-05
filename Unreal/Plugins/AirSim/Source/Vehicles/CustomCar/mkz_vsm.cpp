#include "mkz_vsm.h"
#include "stdio.h"

MkzVsm::MkzVsm() {
    solver.w = float_workspace;
    solver.err = 0;
     // make sure the solver has space for error messages
    solver.buf = error_buffer;

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

    // output variables according to
    // https://docs.google.com/spreadsheets/d/1CHTJ4xOndvNkg14Urvsw0p-Htb5q7pTLX4XYuwyAojE/
    // changed for Sep 26, 2018 model.

    // "Rear Axle" = base_link frame
    // "Chassis" = center of the mass, not in TF
    // Vehicle body
    position.x = output_array[35];    // Global X-Position of Chassis
    position.y = -output_array[36];   // Global Y-Position of Chassis
    position.z = output_array[37];    // Global Z-Position of Chassis
    vehicle_state.position = position;

    orientation.x = output_array[12];  // Angle of the Chassis about the X-axis (Roll Angle)
    orientation.y = output_array[13];  // Angle of the Chassis about the Y-axis (Pitch Angle)
    orientation.z = output_array[14];  // Angle of the Chassis about the Z-axis (Yaw Angle)
    vehicle_state.orientation = orientation;

    velocity.x = output_array[38];    // Longitudinal Velocity of the Chassis
    velocity.y = output_array[39];    // Lateral Velocity of the Chassis
    velocity.z = output_array[40];    // Vertical Velocity of the Chassis
    vehicle_state.velocity = velocity;

    angular_velocity.x = output_array[15];  // Angular Velocity of the Chassis about X-axis (Roll Rate)
    angular_velocity.y = output_array[16];  // Angular Velocity of the Chassis about Y-axis (Pitch Rate)
    angular_velocity.z = output_array[17];  // Angular Velocity of the Chassis about Z-axis (Yaw Rate)
    vehicle_state.angular_velocity = angular_velocity;

    // Wheel states
    fl_ws.angular_velocity = output_array[27];  // Angular Velocity of the Front Left Wheel
    vehicle_state.fl_wheel_state = fl_ws;
    fr_ws.angular_velocity = output_array[34];  // Angular Velocity of the Front Right Wheel
    vehicle_state.fr_wheel_state = fr_ws;
    rl_ws.angular_velocity = output_array[47];  // Angular Velocity of the Rear Left Wheel
    vehicle_state.rl_wheel_state = rl_ws;
    rr_ws.angular_velocity = output_array[54];  // Angular Velocity of the Rear Right Wheel
    vehicle_state.rr_wheel_state = rr_ws;

    vehicle_state.time = this->time;

    return (vehicle_state);
}
