#include "CustomCarPawnApi.h"
#include "AirBlueprintLib.h"

// #include "PhysXVehicleManager.h"

CustomCarPawnApi::CustomCarPawnApi(ACustomCarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, const msr::airlib::GeoPoint& home_geopoint)
    : pawn_(pawn), pawn_kinematics_(pawn_kinematics), home_geopoint_(home_geopoint)
{}

bool CustomCarPawnApi::armDisarm(bool arm)
{
    //TODO: implement arming for car
    unused(arm);
    return true;
}

void CustomCarPawnApi::setCarControls(const CarApiBase::CarControls& controls)
{
    last_controls_ = controls;
    VehicleInput vehicle_input;

    vehicle_input.throttle_percent = controls.throttle;
    vehicle_input.steering_angle = controls.steering * (-8.2);
    vehicle_input.brake_position = controls.brake / 2;

    UE_LOG(LogTemp, Warning, TEXT("Vehicle Brake: %f"), controls.brake);

    pawn_->setVehicleModelInput(vehicle_input);
}

const msr::airlib::CarApiBase::CarControls& CustomCarPawnApi::getCarControls() const
{
    return last_controls_;
}

msr::airlib::CarApiBase::CarState CustomCarPawnApi::getCarState() const
{
    VehicleState vehicle_state = pawn_->getVehicleState();

    CarApiBase::CarState state(
        vehicle_state.velocity.x, // m/s
        1, // Gear
        5.0f, // EngineRotationSpeed
        50.0f, // EngineMaxRotationSpeed
        last_controls_.handbrake,
        *pawn_kinematics_,
        msr::airlib::ClockFactory::get()->nowNanos()
    );
    return state;
}

void CustomCarPawnApi::reset()
{
    msr::airlib::CarApiBase::reset();

    last_controls_ = CarControls();
    auto phys_comps = UAirBlueprintLib::getPhysicsComponents(pawn_);
    UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
        for (auto* phys_comp : phys_comps) {
            phys_comp->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
            phys_comp->SetPhysicsLinearVelocity(FVector::ZeroVector);
            phys_comp->SetSimulatePhysics(false);
        }
        setCarControls(CarControls());
    }, true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
        for (auto* phys_comp : phys_comps)
            phys_comp->SetSimulatePhysics(true);
    }, true);
}

void CustomCarPawnApi::update()
{
    msr::airlib::CarApiBase::update();
}

msr::airlib::GeoPoint CustomCarPawnApi::getHomeGeoPoint() const
{
    return home_geopoint_;
}

void CustomCarPawnApi::enableApiControl(bool is_enabled)
{
    if (api_control_enabled_ != is_enabled) {
        last_controls_ = CarControls();
        api_control_enabled_ = is_enabled;
    }
}

bool CustomCarPawnApi::isApiControlEnabled() const
{
    return api_control_enabled_;
}

CustomCarPawnApi::~CustomCarPawnApi() = default;
