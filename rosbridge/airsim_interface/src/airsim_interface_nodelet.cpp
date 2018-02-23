/* Copyright (c) 2017, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * Waterloo Intelligent Systems Engineering Lab (WISELab),
 * University of Waterloo.
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |                                                                            |
 |                         /\/\__/\_/\      /\_/\__/\/\                       |
 |                         \          \____/          /                       |
 |                          '----________________----'                        |
 |                              /                \                            |
 |                            O/_____/_______/____\O                          |
 |                            /____________________\                          |
 |                           /    (#UNIVERSITY#)    \                         |
 |                           |[**](#OFWATERLOO#)[**]|                         |
 |                           \______________________/                         |
 |                            |_""__|_,----,_|__""_|                          |
 |                            ! !                ! !                          |
 |                            '-'                '-'                          |
 |       __    _   _  _____  ___  __  _  ___  _    _  ___  ___   ____  ____   |
 |      /  \  | | | ||_   _|/ _ \|  \| |/ _ \| \  / |/ _ \/ _ \ /     |       |
 |     / /\ \ | |_| |  | |  ||_||| |\  |||_|||  \/  |||_||||_|| \===\ |====   |
 |    /_/  \_\|_____|  |_|  \___/|_| \_|\___/|_|\/|_|\___/\___/ ____/ |____   |
 |                                                                            |
 ******************************************************************************
 * ############################################################################
 *
 * File: airsim_interface_nodelet.cpp
 * Desc: Implementation of the airsim_interface_nodelet
 * Auth: Ian Colwell
 *
 * ############################################################################
*/

#include "airsim_interface/airsim_interface_nodelet.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <stdlib.h>
#include <string>

namespace airsim_interface {

AirsimInterfaceNodelet::AirsimInterfaceNodelet() {}

void AirsimInterfaceNodelet::onInit() {
    this->nh = getNodeHandle();
    this->private_nh = getPrivateNodeHandle();
    this->loadParams();
    
    // Set nodelet's internal logger
    this->SetInternalLogger();
        
    // Publishers
    // this->rod_constraints_pub = this->nh.advertise<anm_msgs::RODConstraint>(
    //     "rod_constraints", 1);
    
    // this->imu = ImuSimple();

    // Subscribers
    // this->system_health_sub = this->nh.subscribe(
    //   "system_health", 1, &AirsimInterfaceNodelet::readSystemHealthCb, this);
    
    // imu.reset();
    
    // Publisher timer
    this->timer = this->nh.createTimer(ros::Duration(1),
        boost::bind(&AirsimInterfaceNodelet::publishTimerCb, this));
}

void AirsimInterfaceNodelet::loadParams() {
    if (this->nh.getParam("airsim_interface/log_directory", this->log_directory)) {
        ROS_INFO("Loaded log_directory: %s", this->log_directory.c_str());
    } else {
        ROS_INFO("Could not load log_directory");
    }
}

// Set a logger to catch the errors published by the internal C++ library so
// that you don't spam the console!
void AirsimInterfaceNodelet::SetInternalLogger() {
    boost::log::add_common_attributes();

    // Set the logging directory
    std::string log_path = getenv("HOME") + this->log_directory;

    // Filter based on logging severity
    boost::log::core::get()->set_filter(
      // trace and debug level log events are filtered out
      boost::log::trivial::severity >= boost::log::trivial::info);

    // Define the desired log file.
    boost::log::add_file_log(
      // Set log file name
      boost::log::keywords::file_name =
        log_path + "/airsim_interface_nodelet_%N.log",
      // Set log file rotation size in bytes.
      // These values are low here for example.
      boost::log::keywords::rotation_size = 512,
      // Set log entry format
      boost::log::keywords::format = "[%TimeStamp%]: %Message%"
      // Call make_collector off of the sink created by add_file_log
      )
      ->locked_backend()
      ->set_file_collector(
        // Collectors are only applied after a log file is closed.
        // Each time a log file is closed, the collector checks to
        // see if an action needs to be taken relative to the next
        // log file.
        boost::log::sinks::file::make_collector(
          // 'target' sets the folder that will be "managed"
          // by overwriting logs to maintain following
          // objective.
          boost::log::keywords::target = log_path,
          // If the logs being created in total exceed
          // max_size, then the next log file created will
          // overwrite the first log file.
          boost::log::keywords::max_size = 5 * 512));
}


void AirsimInterfaceNodelet::publishTimerCb() {
    ROS_INFO_THROTTLE(1, "Publish timer");
    // imu.update();
    // imu_data = imu.getOutput();
}

// void AirsimInterfaceNodelet::readSystemHealthCb(
//     const anm_msgs::SystemHealth &system_health_msg) {
//     ROS_INFO_THROTTLE(1, "Received System Health message");
// }

}  // namespace airsim_interface

PLUGINLIB_DECLARE_CLASS(airsim_interface,
                        AirsimInterfaceNodelet,
                        airsim_interface::AirsimInterfaceNodelet,
                        nodelet::Nodelet);
