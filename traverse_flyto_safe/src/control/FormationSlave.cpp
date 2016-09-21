/*
 * formationslave.cpp
 *
 *  Created on: Feb 01, 2016
 *      Author: eruff
 */


#include "FormationSlave.h"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>



FormationSlave::FormationSlave()
    : mainNodeHandle( ROSModule::Instance().getMainNodeHandle() )
{
    //FormationElement* fe = new FormationElement(options.tRobotID->getValue(),options.tUseMKInterface->getValue(),
    //                                            options.tUseHumanInput->getValue());
    FormationElement* fe = new FormationElement(&options);
}

FormationSlave::~FormationSlave()
{

}



