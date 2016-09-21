/*
 * FormationControl.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#ifndef FORMATIONCONTROL_HPP_
#define FORMATIONCONTROL_HPP_

#include <telekyb_base/Options.hpp>

#include <sensor_msgs/Joy.h>

#include <telekyb_srvs/BoolInput.h>
#include <telekyb_srvs/StringInput.h>
#include <telekyb_srvs/BehaviorOutput.h>

#include "FormationElement.hpp"


using namespace telekyb;

class FormationControlOptions : public OptionContainer
{
public:
	Option< std::vector<int> >* tRobotIDs;
	Option<std::string>* tJoystickTopic;
	Option<std::string>* tVelocityInputTopic;
	Option<std::string>* tFixPointInputTopic;
	Option<bool>* tUseMKInterface;
	Option<bool>* tUseHumanInput;
	Option< std::vector<int> >* tUsesHumanInput;
	Option<bool>* tDoYawControl;
	Option<bool>* tDoYawFixPointControl;

	Option<double>* tFlyToVelocity;
	Option<double>* tFlyToDestinationRadius;
	Option< std::string >* tFlyToTopic;

	Option<double>* tMaxYawRate;
	
	Option<bool>* tUseJoystick;
	
	FormationControlOptions();
};

class FormationControl {
protected:
	FormationControlOptions options;
	// Elements
	std::vector< FormationElement* > formationElements;

	// ROS
	ros::NodeHandle mainNodeHandle;
	ros::Subscriber joySub;
	ros::Timer timeSub;
	
	ros::ServiceServer clientTakeoffTrigger;
	
	std::vector<int> _robotIDs;
        bool takeOffSignalReceived;
	double timeAtTakeOff;
	bool takeOffDone;
	bool switchedToFormation;

public:
	FormationControl();
	virtual ~FormationControl();

	// setup Behaviors
	void setupFormationControl();

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);
        void timeCB(const ros::TimerEvent&);
	
	bool triggerTakeOff(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

};

#endif /* FORMATIONCONTROL_HPP_ */
