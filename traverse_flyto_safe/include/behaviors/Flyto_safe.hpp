/*
 * Formation.hpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#ifndef FLYTO_SAFE_HPP_
#define FLYTO_SAFE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include "AbstractGraphNode.hpp"

#include <telekyb_base/Spaces/Angle.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// sensormsgs
#include <sensor_msgs/Joy.h>
// Input Velocity
#include <geometry_msgs/Vector3Stamped.h>
// Input YawFixPoint
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <telekyb_msgs/StampedPointArray.h>

#include <obs_avoidance/ObstacleAvoidancePotential.hpp>

// PotentialFunctions
#include <telekyb_calculus/Potentials/CoTanPotentialFunctions.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class Flyto_safe : public AbstractGraphNode {
protected:
	Option< std::vector<int>  >* tNeighbors;
	//Option< std::vector<double> >* tNeighborDistances;

	Option< std::string >* tJoystickTopic;
	Option< bool >* tVelocityInputEnabled;
	Option< std::string >* tVelocityInputTopic;
	Option< std::string >* tYawFixPointInputTopic;

	// flyToOptions
	Position3D tFlyToDestination;
	Position3D tFlyAndLookAtDestination;
	Position3D tCurrentSelfPosition;
	double GazingThresholdDistance;
	Option<double>* tFlyToVelocity;
	Option<double>* tFlyToDestinationRadius;
	Option< std::string >* tFlyToTopic;


	//Option<bool>* tFormationUsePositionMode;
	//Option<double>* tJoystickYawRateScale;

	Option< std::string >* tObsPointsTopicName;
	std::vector<Position3D> lastObstaclePoints;


	Option<double>* tMaxYawRate;
	Option< bool >* tUsesHumanInput;

	// Distance Map to Neighbors
	std::vector<double> distanceVector;

	// Virtual Point of Formation
	Position3D virtualPoint;

	// ROS
	ros::Subscriber userInputSub;
	ros::Subscriber joySub;
	ros::Subscriber obsPointSub;
	ros::Subscriber objectPointSub_;
	ros::Subscriber _flytoPoseSubscriber;

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);
	void userVelocityCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
	void objectPointCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void flytoPoseCallback(const geometry_msgs::Pose::ConstPtr& pose);


	Velocity3D lastVelocityInput;
	//double lastYawRateInput;

	// Integrated Position for Velocity Mode
	//Position3D posModeCurPosition;
	//Angle yawAngle;
	Time posModeLastInputTime;

	// Dead Man
	bool deadManPressed;

	// Outputfield
	bool valid;



	void obsPointCB(const telekyb_msgs::StampedPointArray::ConstPtr& obsPointsMsg);

	ObstacleAvoidancePotential obsPotential;

	// Algorithm related
	Option<double>* tFormationRepulMinDist;
	Option<double>* tFormationAttrGain;
	Option<double>* tFormationRepulGain;
	Option<bool>* tFormationDoYawControl;
	Option<bool>* tLookAtOrigin_wf;
	Option<bool>* tFormationDoYawFixPointControl;// decider if robot does rotate towards one point

	Eigen::Vector3d objectPoint_; // 3D point the robot should look at at all times considering only x/y
	Position3D objectAtOrigin_; //

	double tFormationInitialYawAngle;


	// PotentialFunctions
	std::vector< CoTanRepulsiveGradient*> formationRepulsiveGradientVector;
	std::vector< CoTanAttractiveGradient*> formationAttractiveGradientVector;


	Velocity3D getFormationVirtPointVel();

public:
    Flyto_safe();

	virtual void initialize();
	virtual void destroy();

	//virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Called directly after Change Event is registered.
	virtual bool willBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called after actual Switch. Note: During execution trajectoryStepCreation is used
	virtual void didBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called directly after Change Event is registered: During execution trajectoryStepTermination is used
	virtual void willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);
	// Called after actual Switch. Runs in seperate Thread.
	virtual void didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);

	// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
	virtual void trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
	virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const;
};

}

#endif /* FLYTO_SAFE_HPP_ */
