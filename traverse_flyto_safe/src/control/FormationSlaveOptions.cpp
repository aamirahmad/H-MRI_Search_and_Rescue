#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>

#include "FormationSlaveOptions.hpp"

using namespace telekyb;

// Options
FormationSlaveOptions::FormationSlaveOptions()
    : OptionContainer("FormationSlaveOptions")
{
    tRobotID = addOption<int>("tRobotID","Specifies the ID of the Robot", 0 , false, false);

    tRobotIDs = addOption< std::vector<int> >("tRobotIDs",
                                              "Specifies the ID of the Robots to connect to", std::vector<int>() , false, false);
    tJoystickTopic = addOption<std::string>("tJoystickTopic",
                                            "Joysticktopic to use (sensor_msgs::Joy)", "/TeleKyb/tJoy/joy", false, false);
    tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic",
                                                 "Velocity Input Topic to use (geometry_msgs::Vector3Stamped)",
                                                 "FormationVelocityInput", false, false);
    tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!", false, false, false);
    tUseHumanInput = addOption<bool>("tUseHumanInput",
                                                    "Boollean flag wheather the robot should use the human control input",
                                      false, false, false);
    tUsesHumanInput = addOption< std::vector<int> >("tUsesHumanInput",
                                                    "If the robot should use the human control input", std::vector<int>(), false, false);

}

FormationSlaveOptions::FormationSlaveOptions(const FormationSlaveOptions &opt)
    : OptionContainer("FormationSlaveOptions_2")
{

    tRobotID = addOption<int>("tRobotID","Specifies the ID of the Robot", opt.tRobotID->getValue() , false, false);

    tRobotIDs = addOption< std::vector<int> >("tRobotIDs",
                                              "Specifies the ID of the Robots to connect to",
                                              opt.tRobotIDs->getValue() , false,false);
    tJoystickTopic = addOption<std::string>("tJoystickTopic",
                                            "Joysticktopic to use (sensor_msgs::Joy)",
                                            opt.tJoystickTopic->getValue(), false, false);
    tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic",
                                                 "Velocity Input Topic to use (geometry_msgs::Vector3Stamped)",
                                                 opt.tVelocityInputTopic->getValue(), false, false);
    tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!",
                                      opt.tUseMKInterface->getValue(), false, false);
    tUseHumanInput = addOption<bool>("tUseHumanInput",
                                                    "Boollean flag wheather the robot should use the human control input",
                                      opt.tUseHumanInput->getValue(), false, false);
    tUsesHumanInput = addOption< std::vector<int> >("tUsesHumanInput",
                                                    "If the robot should use the human control input",
                                                    opt.tUsesHumanInput->getValue(), false, false);

}

