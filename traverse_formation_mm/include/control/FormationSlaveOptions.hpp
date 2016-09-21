#ifndef FORMATIONSLAVEOPTIONS_HPP
#define FORMATIONSLAVEOPTIONS_HPP

#include <telekyb_base/Options.hpp>

#include "FormationSlaveOptions.hpp"

using namespace telekyb;

class FormationSlaveOptions : public OptionContainer
{

public:
    Option<int>* tRobotID;
    Option< std::vector<int> >* tRobotIDs;
    Option<std::string>* tJoystickTopic;
    Option<std::string>* tVelocityInputTopic;
    Option<bool>* tUseMKInterface;
    Option<bool>* tUseHumanInput;
    Option< std::vector<int> >* tUsesHumanInput;
    FormationSlaveOptions();
    FormationSlaveOptions(const FormationSlaveOptions &opt);


};

#endif // FORMATIONSLAVEOPTIONS_HPP
