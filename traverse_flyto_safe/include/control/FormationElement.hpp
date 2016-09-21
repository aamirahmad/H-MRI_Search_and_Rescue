/*
 * FormationElement.hpp
 *
 *  Created on: Feb 10, 2012
 *      Author: mriedel
 */

#ifndef FORMATIONELEMENT_HPP_
#define FORMATIONELEMENT_HPP_

#include <ros/ros.h>

#include <telekyb_interface/TeleKybCore.hpp>
#include <telekyb_interface/MKInterface.hpp>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>


#include "FormationSlaveOptions.hpp"

using namespace telekyb;



class FormationElement : telekyb_interface::ActiveBehaviorListener {
private:
    void setupFormationElement();
    bool _useHumanInput;

    FormationSlaveOptions *_formationSlaveOptions;
public:
    // ID
    int robotID;
    ros::NodeHandle _nh;
    ros::Subscriber _flytoPoseSubscriber;
    // the System
    telekyb_interface::TeleKybCore* core;
    // BehaviorController
    telekyb_interface::BehaviorController* bController;
    telekyb_interface::OptionController *oController;



    // Optional MKInterface
    telekyb_interface::MKInterface* mkInterface;

    // Behavior
    telekyb_interface::Behavior* activeBehaviorPtr;


    FormationElement(int robotID_, bool useMKInterface_, bool useHumanInput_);
    FormationElement(FormationSlaveOptions * options);
    virtual ~FormationElement();

    void activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior);

    telekyb_interface::MKInterface* getMKInterfacePointer() const;
    telekyb_interface::TeleKybCore* getTeleKybCorePointer() const;

    // services for convienence
    bool toggleYaw(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool mkSetEmergency(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool mkToggleMotors(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    // lift land
    bool liftland(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool switchToFormation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);


    //	void switchIntoFormation() const;

    // Behaviors
    // System Behaviors
    telekyb_interface::Behavior ground;
    telekyb_interface::Behavior hover;
    telekyb_interface::Behavior normalBreak;
    telekyb_interface::Behavior takeOff;
    telekyb_interface::Behavior land;

    // FormationControl
    telekyb_interface::Behavior bFormation_;

    ros::ServiceServer _toggleMotors_srv;
    ros::ServiceServer _liftland_srv;
    ros::ServiceServer _swToFormation_srv;
    ros::ServiceServer _emergency_srv;
    ros::ServiceServer _toggleYawTo_srv;



};

#endif /* FORMATIONCONTROLELEMENT_HPP_ */
