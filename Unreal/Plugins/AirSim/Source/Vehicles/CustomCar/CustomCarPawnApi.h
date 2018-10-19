#pragma once

#include "vehicles/car/api/CarApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "CustomCarPawn.h"


class CustomCarPawnApi : public msr::airlib::CarApiBase {
public:
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    CustomCarPawnApi(ACustomCarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, const msr::airlib::GeoPoint& home_geopoint,
        std::shared_ptr<msr::airlib::SensorFactory> sensor_factory, const msr::airlib::Kinematics::State& state, const msr::airlib::Environment& environment);

    virtual void setCarControls(const CarApiBase::CarControls& controls) override;

    virtual CarApiBase::CarState getCarState() const override;

    virtual void reset() override;
    virtual void update() override;

    virtual msr::airlib::GeoPoint getHomeGeoPoint() const override;

    virtual void enableApiControl(bool is_enabled) override;
    virtual bool isApiControlEnabled() const override;
    virtual bool armDisarm(bool arm) override;

    virtual const CarApiBase::CarControls& getCarControls() const override;

    virtual ~CustomCarPawnApi();

private:
    bool api_control_enabled_ = false;
    CarControls last_controls_;
    ACustomCarPawn* pawn_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    msr::airlib::GeoPoint  home_geopoint_;
};
