#include "CustomCarPawn.h"
#include "Engine/SkeletalMesh.h"
#include "GameFramework/Controller.h"
#include "Components/TextRenderComponent.h"
#include "Components/AudioComponent.h"
#include "Sound/SoundCue.h"
#include "WheeledVehicleMovementComponent4W.h"

#include "AirBlueprintLib.h"
#include <vector>
#include "common/common_utils/Utils.hpp"
#include "common/ClockFactory.hpp"


#define LOCTEXT_NAMESPACE "VehiclePawn"

ACustomCarPawn::ACustomCarPawn()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;

    // TODO: Change this to some other pawn later
    const auto& car_mesh_paths = AirSimSettings::singleton().pawn_paths["DefaultComputerVision"];

    setupVehicleMovementComponent();

    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

    camera_front_center_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_center_base_"));
    camera_front_center_base_->SetRelativeLocation(FVector(200, 0, 100)); //center
    camera_front_center_base_->SetupAttachment(RootComponent);

    camera_front_left_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_left_base_"));
    camera_front_left_base_->SetRelativeLocation(FVector(200, -12.5, 100)); //left
    camera_front_left_base_->SetupAttachment(RootComponent);

    camera_front_right_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_right_base_"));
    camera_front_right_base_->SetRelativeLocation(FVector(200, 12.5, 100)); //right
    camera_front_right_base_->SetupAttachment(RootComponent);

    camera_driver_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_driver_base_"));
    camera_driver_base_->SetRelativeLocation(FVector(0, -25, 125)); //driver
    camera_driver_base_->SetupAttachment(RootComponent);

    camera_back_center_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_back_center_base_"));
    camera_back_center_base_->SetRelativeLocation(FVector(-400, 0, 200)); //rear
    camera_back_center_base_->SetupAttachment(RootComponent);

    // // In car HUD
    // // Create text render component for in car speed display
    // speed_text_render_ = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarSpeed"));
    // speed_text_render_->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));
    // speed_text_render_->SetRelativeLocation(FVector(35.0f, -6.0f, 20.0f));
    // speed_text_render_->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
    // speed_text_render_->SetupAttachment(GetMesh());
    // speed_text_render_->SetVisibility(true);
    //
    // // Create text render component for in car gear display
    // gear_text_render_ = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarGear"));
    // gear_text_render_->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));
    // gear_text_render_->SetRelativeLocation(FVector(35.0f, 5.0f, 20.0f));
    // gear_text_render_->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
    // gear_text_render_->SetupAttachment(GetMesh());
    // gear_text_render_->SetVisibility(true);

    // Setup the audio component and allocate it a sound cue
    // ConstructorHelpers::FObjectFinder<USoundCue> SoundCue(TEXT("/AirSim/VehicleAdv/Sound/Engine_Loop_Cue.Engine_Loop_Cue"));
    // engine_sound_audio_ = CreateDefaultSubobject<UAudioComponent>(TEXT("EngineSound"));
    // engine_sound_audio_->SetSound(SoundCue.Object);
    // engine_sound_audio_->SetupAttachment(GetMesh());

    // Colors for the in-car gear display. One for normal one for reverse
    last_gear_display_reverse_color_ = FColor(255, 0, 0, 255);
    last_gear_display_color_ = FColor(255, 255, 255, 255);

    is_low_friction_ = false;
}

void ACustomCarPawn::setupVehicleMovementComponent()
{

    FVector init_location = this->GetActorLocation();
    Vector3 init_position;
    init_position.x = init_location.X;
    init_position.y = init_location.Y;
    init_position.z = init_location.Z;
    vehicle_model_.init(init_position);
}

void ACustomCarPawn::setVehicleModelInput(VehicleInput vehicle_input)
{
    vehicle_model_.setVehicleInput(vehicle_input);
}

VehicleState ACustomCarPawn::getVehicleState()
{
    return vehicle_state_;
}

void ACustomCarPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
        HitNormal, NormalImpulse, Hit);
}

void ACustomCarPawn::initializeForBeginPlay(bool engine_sound)
{
    //put camera little bit above vehicle
    FTransform camera_transform(FVector::ZeroVector);
    FActorSpawnParameters camera_spawn_params;
    camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    camera_spawn_params.Name = FName(* (this->GetName() + "_camera_front_center"));
    camera_front_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_center_->AttachToComponent(camera_front_center_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_left"));
    camera_front_left_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_left_->AttachToComponent(camera_front_left_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_right"));
    camera_front_right_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_right_->AttachToComponent(camera_front_right_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_driver"));
    camera_driver_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_driver_->AttachToComponent(camera_driver_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_back_center"));
    camera_back_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_,
        FTransform(FRotator(0, -180, 0), FVector::ZeroVector), camera_spawn_params);
    camera_back_center_->AttachToComponent(camera_back_center_base_, FAttachmentTransformRules::KeepRelativeTransform);

    setupInputBindings();

    manual_pose_controller_ = NewObject<UManualPoseController>(this, "ComputerVision_ManualPoseController");
    manual_pose_controller_->initializeForPlay();
    manual_pose_controller_->setActor(this);
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> ACustomCarPawn::getCameras() const
{
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    cameras.insert_or_assign("front_center", camera_front_center_);
    cameras.insert_or_assign("front_right", camera_front_right_);
    cameras.insert_or_assign("front_left", camera_front_left_);
    cameras.insert_or_assign("fpv", camera_driver_);
    cameras.insert_or_assign("back_center", camera_back_center_);

    cameras.insert_or_assign("0", camera_front_center_);
    cameras.insert_or_assign("1", camera_front_right_);
    cameras.insert_or_assign("2", camera_front_left_);
    cameras.insert_or_assign("3", camera_driver_);
    cameras.insert_or_assign("4", camera_back_center_);

    cameras.insert_or_assign("", camera_front_center_);

    return cameras;
}

void ACustomCarPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    camera_front_center_ = nullptr;
    camera_front_left_ = nullptr;
    camera_front_right_ = nullptr;
    camera_driver_ = nullptr;
    camera_back_center_ = nullptr;

    camera_front_center_base_ = nullptr;
    camera_front_left_base_ = nullptr;
    camera_front_right_base_ = nullptr;
    camera_driver_base_ = nullptr;
    camera_back_center_base_ = nullptr;
}

void ACustomCarPawn::Tick(float Delta)
{
    Super::Tick(Delta);

    // Update the strings used in the HUD (in-car and on-screen)
    updateHUDStrings();

    // Set the string in the in-car HUD
    updateInCarHUD();

    static FVector location = this->GetActorLocation();
    static FRotator rotation = this->GetActorRotation();


    FVector delta_position = FVector::ForwardVector;
    FRotator delta_rotation = FRotator(0,1,0);

    vehicle_model_.performSimulationStep();
    vehicle_state_ = vehicle_model_.getVehicleState();


    FVector new_location = FVector(vehicle_state_.position.x,
                                   vehicle_state_.position.y,
                                   location.Z);

    FRotator new_rotation = FRotator(vehicle_state_.orientation.y,
                                    -vehicle_state_.orientation.z,
                                    vehicle_state_.orientation.x);

    this->SetActorLocationAndRotation(location + rotation.RotateVector(new_location), rotation + new_rotation);

    //update ground level
    if (manual_pose_controller_->getActor() == this) {
        manual_pose_controller_->updateActorPose(Delta);
    }

    pawn_events_.getPawnTickSignal().emit(Delta);
}

void ACustomCarPawn::updateHUDStrings()
{
    // TODO re-enable these logs once done changeover

	float speed_unit_factor = AirSimSettings::singleton().speed_unit_factor;
	FText speed_unit_label = FText::FromString(FString(AirSimSettings::singleton().speed_unit_label.c_str()));
    // float vel = FMath::Abs(GetVehicleMovement()->GetForwardSpeed() / 100); //cm/s -> m/s
    // float vel_rounded = FMath::FloorToInt(vel * 10 * speed_unit_factor) / 10.0f;
    // int32 Gear = GetVehicleMovement()->GetCurrentGear();

    // // Using FText because this is display text that should be localizable
    // last_speed_ = FText::Format(LOCTEXT("SpeedFormat", "{0} {1}"), FText::AsNumber(vel_rounded), speed_unit_label);
    //
    // if (GetVehicleMovement()->GetCurrentGear() < 0)
    // {
    //     last_gear_ = FText(LOCTEXT("ReverseGear", "R"));
    // }
    // else
    // {
    //     last_gear_ = (Gear == 0) ? LOCTEXT("N", "N") : FText::AsNumber(Gear);
    // }
    //
    //
    // UAirBlueprintLib::LogMessage(TEXT("Speed: "), last_speed_.ToString(), LogDebugLevel::Informational);
    // UAirBlueprintLib::LogMessage(TEXT("Gear: "), last_gear_.ToString(), LogDebugLevel::Informational);
    // UAirBlueprintLib::LogMessage(TEXT("RPM: "), FText::AsNumber(GetVehicleMovement()->GetEngineRotationSpeed()).ToString(), LogDebugLevel::Informational);
}

void ACustomCarPawn::updateInCarHUD()
{
    // TODO: Update
    APlayerController* PlayerController = Cast<APlayerController>(GetController());
    // if ((PlayerController != nullptr) && (speed_text_render_ != nullptr) && (gear_text_render_ != nullptr))
    // {
    //     // Setup the text render component strings
    //     speed_text_render_->SetText(last_speed_);
    //     gear_text_render_->SetText(last_gear_);
    //
    //     if (GetVehicleMovement()->GetCurrentGear() >= 0)
    //     {
    //         gear_text_render_->SetTextRenderColor(last_gear_display_color_);
    //     }
    //     else
    //     {
    //         gear_text_render_->SetTextRenderColor(last_gear_display_reverse_color_);
    //     }
    // }
}

/******************* Keyboard bindings*******************/
//This method must be in pawn because Unreal doesn't allow key bindings to non UObject pointers
void ACustomCarPawn::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Up, 1), this,
        this, &ACustomCarPawn::onMoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Down, -1), this,
        this, &ACustomCarPawn::onMoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Right, 0.5), this,
        this, &ACustomCarPawn::onMoveRight);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Left, -0.5), this,
        this, &ACustomCarPawn::onMoveRight);

    UAirBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this, &ACustomCarPawn::onHandbrakePressed, true);
    UAirBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this, &ACustomCarPawn::onHandbrakeReleased, false);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake", EKeys::SpaceBar, 1), this,
        this, &ACustomCarPawn::onFootBrake);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Gamepad_LeftX, 1), this,
        this, &ACustomCarPawn::onMoveRight);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Gamepad_RightTriggerAxis, 1), this,
        this, &ACustomCarPawn::onMoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake", EKeys::Gamepad_LeftTriggerAxis, 1), this,
        this, &ACustomCarPawn::onFootBrake);

    //below is not needed
    //UAirBlueprintLib::BindActionToKey("Reverse", EKeys::Down, this, &ACustomCarPawn::onReversePressed, true);
    //UAirBlueprintLib::BindActionToKey("Reverse", EKeys::Down, this, &ACustomCarPawn::onReverseReleased, false);
}

void ACustomCarPawn::onMoveForward(float Val)
{
    if (Val < 0)
        onReversePressed();
    else
        onReverseReleased();

    keyboard_controls_.throttle = Val;
}

void ACustomCarPawn::onMoveRight(float Val)
{
    keyboard_controls_.steering = Val;
}

void ACustomCarPawn::onHandbrakePressed()
{
    keyboard_controls_.handbrake = true;
}

void ACustomCarPawn::onHandbrakeReleased()
{
    keyboard_controls_.handbrake = false;
}

void ACustomCarPawn::onFootBrake(float Val)
{
    keyboard_controls_.brake = Val;
}

void ACustomCarPawn::onReversePressed()
{
    if (keyboard_controls_.manual_gear >= 0) {
        keyboard_controls_.is_manual_gear = true;
        keyboard_controls_.manual_gear = -1;
        keyboard_controls_.gear_immediate = true;
    }
}

void ACustomCarPawn::onReverseReleased()
{
    if (keyboard_controls_.manual_gear < 0) {
        keyboard_controls_.is_manual_gear = false;
        keyboard_controls_.manual_gear = 0;
        keyboard_controls_.gear_immediate = true;
    }
}

#undef LOCTEXT_NAMESPACE
