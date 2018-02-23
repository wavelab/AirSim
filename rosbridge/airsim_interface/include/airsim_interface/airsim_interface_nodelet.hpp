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
 * File: airsim_interface_nodelet.hpp
 * Desc: Nodelet header for the airsim_interface.
 * Auth: Ian Colwell
 *
 * ############################################################################
*/

#ifndef airsim_interface_AIRSIMINTERFACENODELET_H
#define airsim_interface_AIRSIMINTERFACENODELET_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

#include <string>
#include <iostream>

#include <sensors/imu/ImuSimple.hpp>

namespace airsim_interface {

class AirsimInterfaceNodelet : public nodelet::Nodelet {
 public:
    AirsimInterfaceNodelet();

 private:
    virtual void onInit();

    void loadParams();

    // Set logging sink to be used by boost::log.
    //      Only used for code internal to the nodelet (non-ROS code)
    void SetInternalLogger();
    
    void publishTimerCb();

    // Nodehandles, both public and private
    ros::NodeHandle nh, private_nh;

    // Publishers
    ros::Publisher rod_constraints_pub;
    
    // Subscribers
    ros::Subscriber system_health_sub;

    // Log directory relative to the users $HOME directory
    std::string log_directory;
    
    msr::airlib::ImuSimple imu;
    msr::airlib::ImuBase::Output imu_data;
    
    ros::Timer timer;
    
    
};

}  // namespace airsim_interface
#endif  // airsim_interface_AIRSIMINTERFACENODELET_H
