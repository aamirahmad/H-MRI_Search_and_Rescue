#ifndef FORMATIONSLAVE_H
#define FORMATIONSLAVE_H

#include "telekyb_base/Options.hpp"

#include "FormationElement.hpp"
#include "FormationSlaveOptions.hpp"

using namespace telekyb;

class FormationSlave{

protected:
    FormationSlaveOptions options;
    // ROS
    ros::NodeHandle mainNodeHandle;

public:
    FormationSlave();

    virtual ~FormationSlave();



};

#endif // FORMATIONSLAVE_H
