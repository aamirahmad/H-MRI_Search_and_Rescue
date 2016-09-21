/*
 * Main.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */
#include <telekyb_base/TeleKyb.hpp>
#include <ros/ros.h>

#include "FormationSlave.h"

int main(int argc, char **argv) {

    // disable
    telekyb::RawOptionsContainer::addOption("tRosNrSpinnerThreads","4");
    telekyb::TeleKyb::init(argc,argv, "tk_formation_slave");

    FormationSlave* e = new FormationSlave();

    // spin here
    ros::spin();

    delete e;

    telekyb::TeleKyb::shutdown();

}


