// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/car/api/CarRpcLibClient.hpp"

#include "MavLinkVehicle.hpp"
#include "MavLinkConnection.hpp"
#include "MavLinkMessages.hpp"
#include "MavLinkNode.hpp"
#include "MavLinkVideoStream.hpp"

#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include <thread>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#undef check
#ifdef nil
#undef nil
#endif // nil
#include "rpc/client.h"
#include "vehicles/car/api/CarRpcLibAdapators.hpp"
STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning( disable : 4239))
#endif	



namespace msr { namespace airlib {


typedef msr::airlib_rpclib::CarRpcLibAdapators CarRpcLibAdapators;

CarRpcLibClient::CarRpcLibClient(const string&  ip_address, uint16_t port, uint timeout_ms): RpcLibClientBase(ip_address, port, timeout_ms)
{
}

CarRpcLibClient::~CarRpcLibClient()
{}

void CarRpcLibClient::setCarControls(const CarApiBase::CarControls& controls)
{
    static_cast<rpc::client*>(getClient()) -> call("setCarControls", CarRpcLibAdapators::CarControls(controls));
}

CarApiBase::CarState CarRpcLibClient::getCarState()
{
    return static_cast<rpc::client*>(getClient()) -> call("getCarState").as<CarRpcLibAdapators::CarState>().to();
}

GeoPoint CarRpcLibClient::getGpsLocation()
{
    return GeoPoint(99.0, 99.0, 99.0); //static_cast<rpc::client*>(getClient()) -> call("getGpsLocation").as<CarRpcLibAdapators::GeoPoint>().to();
}

/*
void sendDistanceSensor(float min_distance, float max_distance, float current_distance, float sensor_type, float sensor_id, float orientation)
{
    if (!is_simulation_mode_)
        throw std::logic_error("Attempt to send simulated distance sensor messages while not in simulation mode");

    mavlinkcom::MavLinkDistanceSensor distance_sensor;
    distance_sensor.time_boot_ms = static_cast<uint32_t>(Utils::getTimeSinceEpochNanos() / 1000000.0);

    distance_sensor.min_distance = static_cast<uint16_t>(min_distance);
    distance_sensor.max_distance = static_cast<uint16_t>(max_distance);
    distance_sensor.current_distance = static_cast<uint16_t>(current_distance);
    distance_sensor.type = static_cast<uint8_t>(sensor_type);
    distance_sensor.id = static_cast<uint8_t>(sensor_id);
    distance_sensor.orientation = static_cast<uint8_t>(orientation);
    //TODO: use covariance parameter?

    if (hil_node_ != nullptr) {
        hil_node_->sendMessage(distance_sensor);
    }

    std::lock_guard<std::mutex> guard(last_message_mutex_);
    last_distance_message_ = distance_sensor;
}

void sendHILGps(const GeoPoint& geo_point, const Vector3r& velocity, float velocity_xy, float cog, float eph, float epv, int fix_type, unsigned int satellites_visible)
{
    if (!is_simulation_mode_)
        throw std::logic_error("Attempt to send simulated GPS messages while not in simulation mode");

    mavlinkcom::MavLinkHilGps hil_gps;
    hil_gps.time_usec = static_cast<uint64_t>(Utils::getTimeSinceEpochNanos() / 1000.0);
    hil_gps.lat = static_cast<int32_t>(geo_point.latitude * 1E7);
    hil_gps.lon = static_cast<int32_t>(geo_point.longitude* 1E7);
    hil_gps.alt = static_cast<int32_t>(geo_point.altitude * 1000);
    hil_gps.vn = static_cast<int16_t>(velocity.x() * 100);
    hil_gps.ve = static_cast<int16_t>(velocity.y() * 100);
    hil_gps.vd = static_cast<int16_t>(velocity.z() * 100);
    hil_gps.eph = static_cast<uint16_t>(eph * 100);
    hil_gps.epv = static_cast<uint16_t>(epv * 100);
    hil_gps.fix_type = static_cast<uint8_t>(fix_type);
    hil_gps.vel = static_cast<uint16_t>(velocity_xy * 100);
    hil_gps.cog = static_cast<uint16_t>(cog * 100);
    hil_gps.satellites_visible = static_cast<uint8_t>(satellites_visible);

    if (hil_node_ != nullptr) {
        hil_node_->sendMessage(hil_gps);
    }

    if (hil_gps.lat < 0.1f && hil_gps.lat > -0.1f) {
        //Utils::DebugBreak();
        Utils::log("hil_gps.lat was too close to 0", Utils::kLogLevelError);
    }

    std::lock_guard<std::mutex> guard(last_message_mutex_);
    last_gps_message_ = hil_gps;
}


void updateState()
{
    StatusLock lock(parent_);
    if (mav_vehicle_ != nullptr) {
        int version = mav_vehicle_->getVehicleStateVersion();
        if (version != state_version_)
        {
            current_state = mav_vehicle_->getVehicleState();
            state_version_ = version;
        }
    }
}
*/

/*
GeoPoint getHomeGeoPoint()
{
    updateState();
    if (current_state.home.is_set)
        return GeoPoint(current_state.home.global_pos.lat, current_state.home.global_pos.lon, current_state.home.global_pos.alt);
    else
        return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
}

GeoPoint getGpsLocation()
{
    updateState();
    return GeoPoint(current_state.global_est.pos.lat, current_state.global_est.pos.lon, current_state.global_est.pos.alt);
}
*/

}} //namespace

#endif
#endif
