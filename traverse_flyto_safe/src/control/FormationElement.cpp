/*
 * FormationElement.cpp
 *
 *  Created on: Feb 10, 2012
 *      Author: mriedel
 */

#include "FormationElement.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>

bool toggleMotors(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    return true;
}

void callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


FormationElement::FormationElement(FormationSlaveOptions *options)
    : robotID(options->tRobotID->getValue()), core(NULL), mkInterface(NULL), _nh( ROSModule::Instance().getMainNodeHandle())
{
    _formationSlaveOptions = options;
    core = telekyb_interface::TeleKybCore::getTeleKybCore(robotID);
    if (!core) {
        // fail
        ros::shutdown();
        return;
    }

    //options.tUseMKInterface->setValue(true);
    if (_formationSlaveOptions->tUseMKInterface->getValue()) {
        ROS_INFO("Creating MKInterface!");
        // use MKInterface
        mkInterface = telekyb_interface::MKInterface::getMKInterface(robotID, 5.0);
        if (!mkInterface) {
            // fail
            ros::shutdown();
            return;
        }
    }

    bController = core->getBehaviorController();
    oController = core->getOptionController();


    //activeBehavior = bController->getActiveBehaviorReference();
    bController->setActiveBehaviorListener(this);
    activeBehaviorPtr = bController->getActiveBehaviorPointer();
    setupFormationElement();
}

FormationElement::FormationElement(int robotID_, bool useMKInterface_, bool useHumanInput_)
    : robotID(robotID_), core(NULL), mkInterface(NULL), _nh( ROSModule::Instance().getMainNodeHandle()),
      _useHumanInput(useHumanInput_)
{
	core = telekyb_interface::TeleKybCore::getTeleKybCore(robotID);
	if (!core) {
		// fail
		ros::shutdown();
		return;
	}

	//options.tUseMKInterface->setValue(true);
    if (useMKInterface_) {
		ROS_INFO("Creating MKInterface!");
		// use MKInterface
		mkInterface = telekyb_interface::MKInterface::getMKInterface(robotID, 5.0);
		if (!mkInterface) {
			// fail
			ros::shutdown();
			return;
		}
	}

	bController = core->getBehaviorController();
	oController = core->getOptionController();

	//activeBehavior = bController->getActiveBehaviorReference();
	bController->setActiveBehaviorListener(this);
	activeBehaviorPtr = bController->getActiveBehaviorPointer();

    setupFormationElement();
}

FormationElement::~FormationElement()
{
	if (mkInterface) { delete mkInterface; }

	delete core;
}


void FormationElement::setupFormationElement()
{
	// load Behaviors
	ground = bController->getSystemBehavior("tk_behavior/Ground");
	hover = bController->getSystemBehavior("tk_behavior/Hover");
	normalBreak = bController->getSystemBehavior("tk_behavior/NormalBrake");
	takeOff = bController->getSystemBehavior("tk_behavior/TakeOff");
	land = bController->getSystemBehavior("tk_behavior/Land");


	// sanity check
	if (ground.isNull() || hover.isNull() || normalBreak.isNull() || takeOff.isNull() || land.isNull()) {
		ROS_FATAL("Unable to get SystemBehavior for UAV %d !!!", robotID);
		//ROS_BREAK();
		ros::shutdown();
	}

	// setup takeoff
	//takeOff.getOptionContainer().getOption("tTakeOffDestination").set(Position3D(0.0,0.0,-1.0));
	takeOff.getOptionContainer().getOption("tTakeOffVertically").set(false);

	// done
	takeOff.setParameterInitialized(true);

	//land.getOptionContainer().getOption("tLandDestination").set(Position3D(0.0,0.0,0.0));
	land.getOptionContainer().getOption("tLandVertically").set(true);
	land.setParameterInitialized(true);


    bFormation_ = bController->loadBehavior("traverse_flyto_safe/Flyto_safe");
    bFormation_.getOptionContainer().getOption("tLookAtOrigin_wf").set(true);

    if ( bFormation_.isNull()) {
		ROS_FATAL("Unable to load FormationControl for UAV %d !!!", robotID);
		//ROS_BREAK();
		ros::shutdown();
	}


	// configure calibrator
	//formation.setNextBehavior(land);
	//formation.setParameterInitialized(true);


	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV %d not in Ground Behavior during Startup", robotID);
		ros::shutdown();
	}

    ROS_INFO_STREAM("init the services for mulitmaster setup");
    // these services are used to communicate with the formations "slaves"
    _toggleMotors_srv = _nh.advertiseService("ToggleMotors", &FormationElement::mkToggleMotors,this);
    _liftland_srv = _nh.advertiseService("LiftLand", &FormationElement::liftland,this);
    _swToFormation_srv = _nh.advertiseService("SwtichToFormation", &FormationElement::switchToFormation,this);
    _emergency_srv = _nh.advertiseService("Emergency", &FormationElement::mkSetEmergency,this);
    _toggleYawTo_srv = _nh.advertiseService("ToggleYaw", &FormationElement::toggleYaw,this);
}

bool FormationElement::toggleYaw(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    bool lookAtYaw;
    bFormation_.getOptionContainer().getOption("tLookAtOrigin_wf").get(lookAtYaw);
    if(lookAtYaw){
        bFormation_.getOptionContainer().getOption("tLookAtOrigin_wf").set(false);
    }else{
        bFormation_.getOptionContainer().getOption("tLookAtOrigin_wf").set(true);
    }
    res.success = true;
    return res.success;
}


telekyb_interface::MKInterface* FormationElement::getMKInterfacePointer() const
{
	return mkInterface;
}

telekyb_interface::TeleKybCore* FormationElement::getTeleKybCorePointer() const
{
	return core;
}

bool FormationElement::mkSetEmergency(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	if (mkInterface) {
        res.success = mkInterface->setEmergency();
        return res.success;
	}
	return false;
}

bool FormationElement::mkToggleMotors(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	// Button 1! toggle Motors for mkInterface Only in Ground
	if (mkInterface && *activeBehaviorPtr == ground) {
		ROS_INFO("Toggle Motorstate!");

		MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
		if (!mkInterface->updateMKValue(motorState)) {
			ROS_ERROR("Could not get Motorstate!");
			return false;
		}

		if (motorState.value == MotorState::On) {
			// stop
			motorState.value = MotorState::Off;
            res.success = mkInterface->setMKValue(motorState);
            return res.success;
		} else if (motorState.value == MotorState::Off) {
			// start
			motorState.value = MotorState::Init;
            res.success = mkInterface->setMKValue(motorState);
            return res.success;
		} else if (motorState.value == MotorState::Init) {
			motorState.value = MotorState::On;
            res.success = mkInterface->setMKValue(motorState);
            return res.success;
		}
	}
	return false;
}

bool FormationElement::liftland(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ROS_WARN_STREAM("LiftLand " << _nh.getNamespace());
    if (*activeBehaviorPtr ==  ground) {

		if (mkInterface) {
			MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
			if (!mkInterface->updateMKValue(motorState)) {
				ROS_ERROR("Could not get Motorstate!");
				return false;
			}
			if (motorState.value == MotorState::On) {
				bController->switchBehavior(takeOff);
			} else {
				ROS_ERROR("Motors have to be on for liftOff!");
				return false;
			}
		}
		// takeoff
		bController->switchBehavior(takeOff);
        res.success = true;
        return res.success;

	} else {
		// flying -> land
		bController->switchBehavior(land);
        res.success = true;
        return res.success;
	}

	return false;
}

bool FormationElement::switchToFormation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    bController->switchBehavior(bFormation_);
    res.success = true;
    return res.success;

}

void FormationElement::activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior)
{
	if (mkInterface && newActiveBehavior == ground) {
		MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE,0);
		motorState.value = MotorState::Off;
		mkInterface->setMKValue(motorState);
	}
}

//telekyb_interface::Behavior FormationElement::getFormationBehavior() const
//{
//	return formation;
//}

//void FormationElement::switchIntoFormation() const
//{
//	bController->switchBehavior(formation);
//}






