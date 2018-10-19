#include "CustomCarPawn.h"
#include "Engine/SkeletalMesh.h"
#include "GameFramework/Controller.h"
#include "Components/TextRenderComponent.h"
#include "Components/AudioComponent.h"
#include "Sound/SoundCue.h"
#include "WheeledVehicleMovementComponent4W.h"

#include "GameFramework/RotatingMovementComponent.h"

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
    camera_back_center_base_->SetRelativeLocation(FVector(-700, 0, 200)); //rear
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
    vehicle_model_.init();
}

void ACustomCarPawn::setVehicleModelInput(VehicleInput vehicle_input)
{
    vehicle_model_.setVehicleInput(vehicle_input);
    float steering = FMath::Clamp((float)vehicle_input.steering_angle, -8.48f, 8.48f); 
    tire_angle_ = (steering/14.8f)*RAD2DEG*-1;
    UE_LOG(LogTemp, Warning, TEXT("Tire Angle: %f"), tire_angle_);
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

    rotating_movement_fl_ = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("wheel_fl_rotation"));
    rotating_movement_fr_ = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("wheel_fr_rotation"));
    rotating_movement_rl_ = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("wheel_rl_rotation"));
    rotating_movement_rr_ = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("wheel_rr_rotation"));

    wheel_fl_ = UAirBlueprintLib::GetActorComponent<UStaticMeshComponent>(this, TEXT("wheel_flMesh"));
    wheel_fr_ = UAirBlueprintLib::GetActorComponent<UStaticMeshComponent>(this, TEXT("wheel_frMesh"));
    wheel_rl_ = UAirBlueprintLib::GetActorComponent<UStaticMeshComponent>(this, TEXT("wheel_rlMesh"));
    wheel_rr_ = UAirBlueprintLib::GetActorComponent<UStaticMeshComponent>(this, TEXT("wheel_rrMesh"));

    setupInputBindings();

    this->ros_node_wrapper.Reset(
        unreal_ros_node_wrapper::UnrealRosNodeWrapper::create());

    if (this->ros_node_wrapper->start()) {
      UE_LOG(LogTemp, Display, TEXT("unreal_ros_node_wrapper initialized"));
    } else {
      UE_LOG(LogTemp, Warning, TEXT("unreal_ros_node_wrapper not initialized"));
    }
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

VehiclePose ACustomCarPawn::updateVehicleModel()
{
    // store the initial location and rotation of the car when the scenario starts
    static FVector location = this->GetActorLocation();
    static FRotator rotation = this->GetActorRotation();

    vehicle_model_.performSimulationStep();
    vehicle_state_ = vehicle_model_.getVehicleState();

    const double position[3] = {vehicle_state_.position.x,
                                vehicle_state_.position.y,
                                vehicle_state_.position.z};
    const double rotation_matrix[9] = {vehicle_state_.rotation.r1c1,
                                vehicle_state_.rotation.r2c1,
                                vehicle_state_.rotation.r3c1,
                                vehicle_state_.rotation.r1c2,
                                vehicle_state_.rotation.r2c2,
                                vehicle_state_.rotation.r3c2,
                                vehicle_state_.rotation.r1c3,
                                vehicle_state_.rotation.r2c3,
                                vehicle_state_.rotation.r3c3};

    this->ros_node_wrapper->broadcast_T_sim_NED_sim_base_link(position, rotation_matrix);
    this->ros_node_wrapper->publish_wheel_speeds(vehicle_state_.fl_wheel_state.angular_velocity,
                                                 vehicle_state_.fr_wheel_state.angular_velocity,
                                                 vehicle_state_.rl_wheel_state.angular_velocity,
                                                 vehicle_state_.rr_wheel_state.angular_velocity);

    FVector new_location = FVector(vehicle_state_.position.x * 100, // convert output from m to cm
                                   vehicle_state_.position.y * 100, // convert output from m to cm
                                   location.Z);

    FRotator new_rotation = FRotator(vehicle_state_.orientation.y * RAD2DEG, //convert radian to degree
                                    -vehicle_state_.orientation.z * RAD2DEG, //convert radian to degree
                                     vehicle_state_.orientation.x * RAD2DEG);//convert radian to degree

    VehiclePose newPose;
    newPose.location = location + rotation.RotateVector(new_location);
    newPose.rotation = rotation + new_rotation;

    return newPose;
}

void ACustomCarPawn::Tick(float Delta)
{
    Super::Tick(Delta);

    // Update the strings used in the HUD (in-car and on-screen)
    updateHUDStrings();

    // Set the string in the in-car HUD
    updateInCarHUD();

    // Update the vehicle model state
    VehiclePose newPose = updateVehicleModel();

    // Set the location and rotation of the carpawn in simulation
    this->SetActorLocationAndRotation(newPose.location, newPose.rotation);

    pawn_events_.getPawnTickSignal().emit(Delta);

    if (even) { // Set tire angle
        FRotator rot_left(0,tire_angle_,0);
        FRotator rot_right(0,180+tire_angle_, 0);
        wheel_fl_->SetRelativeRotation(rot_left, false, NULL, ETeleportType::None);
        wheel_fr_->SetRelativeRotation(rot_right, false, NULL, ETeleportType::None);
        even = false;
    }
    else { // Set tire speed
        rotating_movement_fl_->RotationRate.Roll = vehicle_state_.fl_wheel_state.angular_velocity*RAD2DEG;
        rotating_movement_fr_->RotationRate.Roll = vehicle_state_.fr_wheel_state.angular_velocity*RAD2DEG*-1;
        rotating_movement_rl_->RotationRate.Roll = vehicle_state_.rl_wheel_state.angular_velocity*RAD2DEG;
        rotating_movement_rr_->RotationRate.Roll = vehicle_state_.rr_wheel_state.angular_velocity*RAD2DEG*-1;

        rotating_movement_fl_->SetUpdatedComponent(wheel_fl_);
        rotating_movement_fr_->SetUpdatedComponent(wheel_fr_);
        rotating_movement_rl_->SetUpdatedComponent(wheel_rl_);
        rotating_movement_rr_->SetUpdatedComponent(wheel_rr_);
        even = true;
    }
}

void ACustomCarPawn::updateHUDStrings()
{
    // TODO re-enable these logs to use vehicle model info

	float speed_unit_factor = AirSimSettings::singleton().speed_unit_factor;
	FText speed_unit_label = FText::FromString(FString(AirSimSettings::singleton().speed_unit_label.c_str()));
    // float vel = FMath::Abs(GetVehicleMovement()->GetForwardSpeed() / 100); //cm/s -> m/s
    // float vel_rounded = FMath::FloorToInt(vel * 10 * speed_unit_factor) / 10.0f;
    // int32 Gear = GetVehicleMovement()->GetCurrentGear();

    // Using FText because this is display text that should be localizable
    // last_speed_ = FText::Format(LOCTEXT("SpeedFormat", "{0} {1}"), FText::AsNumber(vel_rounded), speed_unit_label);
    
    // if (GetVehicleMovement()->GetCurrentGear() < 0)
    // {
    //    last_gear_ = FText(LOCTEXT("ReverseGear", "R"));
    // }
    // else
    // {
    //    last_gear_ = (Gear == 0) ? LOCTEXT("N", "N") : FText::AsNumber(Gear);
    // }
    
    
     //UAirBlueprintLib::LogMessage(TEXT("Speed: "), last_speed_.ToString(), LogDebugLevel::Informational);
     //UAirBlueprintLib::LogMessage(TEXT("Gear: "), last_gear_.ToString(), LogDebugLevel::Informational);
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
