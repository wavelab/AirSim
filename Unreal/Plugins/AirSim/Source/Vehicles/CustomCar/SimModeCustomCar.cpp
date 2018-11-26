#include "SimModeCustomCar.h"
#include "ConstructorHelpers.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "CustomCarPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "vehicles/car/api/CarRpcLibServer.hpp"


void ASimModeCustomCar::BeginPlay()
{
    Super::BeginPlay();

    //initializePauseState();
}

void ASimModeCustomCar::initializePauseState()
{
    pause_period_ = 0;
    pause_period_start_ = 0;
    pause(false);
}

bool ASimModeCustomCar::isPaused() const
{
    return current_clockspeed_ == 0;
}

void ASimModeCustomCar::pause(bool is_paused)
{
/*
    if (is_paused)
        current_clockspeed_ = 0;
    else
        current_clockspeed_ = getSettings().clock_speed;

    UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
*/
}

void ASimModeCustomCar::continueForTime(double seconds)
{
/*
    pause_period_start_ = ClockFactory::get()->nowNanos();
    pause_period_ = seconds;
    pause(false);
*/
}

void ASimModeCustomCar::setupClockSpeed()
{
/*
    current_clockspeed_ = getSettings().clock_speed;

    //setup clock in PhysX
    UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
    UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(current_clockspeed_), LogDebugLevel::Informational);
*/
}

void ASimModeCustomCar::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
/*
    if (pause_period_start_ > 0) {
        if (ClockFactory::get()->elapsedSince(pause_period_start_) >= pause_period_) {
            if (!isPaused())
                pause(true);

            pause_period_start_ = 0;
        }
    }
*/
}

//-------------------------------- overrides -----------------------------------------------//

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeCustomCar::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::CarRpcLibServer(
        getApiProvider(), getSettings().api_server_address));
#endif
}

void ASimModeCustomCar::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
    UAirBlueprintLib::FindAllActor<TPawn>(this, pawns);
}

bool ASimModeCustomCar::isVehicleTypeSupported(const std::string& vehicle_type) const
{
    return vehicle_type == AirSimSettings::kVehicleTypeCustomCar;
}

std::string ASimModeCustomCar::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
    //decide which derived BP to use
    std::string pawn_path = vehicle_setting.pawn_path;
    // TODO: fix this path
    if (pawn_path == "")
        pawn_path = "CustomCar";



    return pawn_path;
}

PawnEvents* ASimModeCustomCar::getVehiclePawnEvents(APawn* pawn) const
{
    return static_cast<TPawn*>(pawn)->getPawnEvents();
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeCustomCar::getVehiclePawnCameras(
    APawn* pawn) const
{
    return (static_cast<const TPawn*>(pawn))->getCameras();
}
void ASimModeCustomCar::initializeVehiclePawn(APawn* pawn)
{
    UE_LOG(LogTemp, Warning, TEXT("SimModeCustomCar"));
    auto vehicle_pawn = static_cast<TPawn*>(pawn);
    vehicle_pawn->initializeForBeginPlay(getSettings().engine_sound);
}
std::unique_ptr<PawnSimApi> ASimModeCustomCar::createVehicleSimApi(
    const PawnSimApi::Params& pawn_sim_api_params) const
{
    auto vehicle_pawn = static_cast<TPawn*>(pawn_sim_api_params.pawn);
    auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new CustomCarPawnSimApi(pawn_sim_api_params,
        vehicle_pawn->getKeyBoardControls()));
    vehicle_sim_api->reset();
    return vehicle_sim_api;
}
msr::airlib::VehicleApiBase* ASimModeCustomCar::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
    const PawnSimApi* sim_api) const
{
    const auto car_sim_api = static_cast<const CustomCarPawnSimApi*>(sim_api);
    return car_sim_api->getVehicleApi();
}
