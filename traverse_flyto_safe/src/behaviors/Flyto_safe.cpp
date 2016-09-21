/*
 * Formation.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#include "Flyto_safe.hpp"

#include <telekyb_base/ROS.hpp>

#include <geometry_msgs/PointStamped.h>

#include "Neighbor.hpp"

#include <telekyb_base/Time.hpp>

#define INTERNAL_SUB_STEP 0.001

// Declare
PLUGINLIB_DECLARE_CLASS(traverse_flyto_safe, Flyto_safe, telekyb_behavior::Flyto_safe, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

Flyto_safe::Flyto_safe()
    : AbstractGraphNode("traverse_flyto_safe/Flyto_safe", BehaviorType::Air),
      obsPotential("ID/" + getIDString() + "/FormationObstaclePotential")
{
    tFormationInitialYawAngle = 0.0;
}

void Flyto_safe::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (msg->buttons.size() < 1) {
        ROS_ERROR("Joystick Message does not have enough Buttons");
    }

    // Dead Man Button
    deadManPressed = (bool)msg->buttons[0];

    if ( msg->buttons[1] ) {
        valid = false;
    }
}

void Flyto_safe::userVelocityCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    // no dead man button
    if (deadManPressed) {
        lastVelocityInput = Velocity3D(msg->vector.x, msg->vector.y, msg->vector.z);
    } else {
        lastVelocityInput = Velocity3D::Zero();
        //lastYawRateInput = 0.0;
    }
}

void Flyto_safe::obsPointCB(const telekyb_msgs::StampedPointArray::ConstPtr& obsPointsMsg)
{
    //	ROS_INFO("Received Obstacle Points");
    //boost::mutex::scoped_lock lastObstaclePointsLock(lastObstaclePointsMutex);
    lastObstaclePoints.resize(obsPointsMsg->points.size());

    for (unsigned int i = 0; i < lastObstaclePoints.size(); ++i) {
        lastObstaclePoints[i](0) = obsPointsMsg->points[i].x;
        lastObstaclePoints[i](1) = obsPointsMsg->points[i].y;
        lastObstaclePoints[i](2) = obsPointsMsg->points[i].z;
    }
}

/**
 * @brief Formation::yawFixPointCB callback function to get the point to which the robot
 * should orinent it self towards if yaw fix point ctrl is active
 * @param msg
 */

void Flyto_safe::objectPointCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    objectPoint_(0) = msg->pose.pose.position.x;
    objectPoint_(1) = msg->pose.pose.position.y;
    objectPoint_(2) = msg->pose.pose.position.z;

    if(tLookAtOrigin_wf->getValue()){
        objectPoint_ = objectAtOrigin_;
    }

}

void Flyto_safe::flytoPoseCallback(const geometry_msgs::Pose::ConstPtr &pose){
    // setting the next fly to position
    printf("fly to safe plugin fly to callback \n\n\n\n\n\n");
    
    tFlyAndLookAtDestination = telekyb::Position3D(pose->position.x,
						   pose->position.y,
						   pose->position.z);
    
    //Calculating tFlyToDestination from tCurrentSelfPosition and tFlyAndLookAtDestination. tFlyToDestination is the point on a circle centered at tFlyAndLookAtDestination but at a radius of GazingThresholdDistance in the direction of flight from tCurrentSelfPosition
    double R = GazingThresholdDistance;
    double m= (tFlyAndLookAtDestination(1)-tCurrentSelfPosition(1))/(tFlyAndLookAtDestination(0)-tCurrentSelfPosition(0));
    double x_a = tFlyAndLookAtDestination(0) - (R)/(powf((1+m*m),0.5));
    double y_a = tFlyAndLookAtDestination(1) - (R*m)/(powf((1+m*m),0.5));
    
    tFlyToDestination = telekyb::Position3D(x_a,
					    y_a,
					    pose->position.z);    
    
    
    ROS_WARN_STREAM("new pose " << pose);
}


void Flyto_safe::initialize()
{
  
    printf("fly to safe plugin initialized \n\n\n\n\n\n");
  
    ///@hack : Put this as a parameter in the parameter server someday
    GazingThresholdDistance = 0.5;
    
    tNeighbors = addOption<std::vector<int> >("tNeighbors",
                                              "Neighbors",
                                              std::vector<int>(), false, false);
    //	tNeighborDistances = addOption<std::vector<double> >("tNeighborDistances",
    //			"Distances to Neighbors",
    //			std::vector<double>(), false, false);


    tJoystickTopic = addOption<std::string>("tJoystickTopic",
                                            "FormationTopic that published sensor_msgs::Joy",
                                            "/TeleKyb/tJoy/joy", false, false);


    tVelocityInputEnabled = addOption<bool>("tVelocityInputEnabled",
                                            "This formation element receives commands.",
                                            false, false, false);

    tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic", "Topic Name of Input of User Velocity",
                                                 "undef", false, false);
    tYawFixPointInputTopic = addOption<std::string>("tYawFixPointInputTopic", "Topic Name for the FixPoint yaw ctrl",
                                                    "/fixPoint", false, false);
    //tFormationUsePositionMode = addOption("tFormationUsePositionMode", "Integrates Position from Velocity input.", true, false, false);
    //tJoystickYawRateScale = addOption("tFormationYawRateScale","Commanded Yaw Rate is scaled to this value", 1.0, false ,false);


    tObsPointsTopicName = addOption<std::string>("tObsPointsTopicName", "Topic Name of Obstacle Points",
                                                 "undef", true, false);


    tFormationRepulMinDist = addOption<double>("tFormationRepulMinDist",
                                               "Distances to Neighbors",
                                               0.7, false, false);
    tFormationAttrGain = addOption<double>("tFormationAttrGain",
                                           "Distances to Neighbors",
                                           0.2, false, false);
    tFormationRepulGain = addOption<double>("tFormationRepulGain",
                                            "Distances to Neighbors",
                                            2.0, false, false);
    tFormationDoYawControl = addOption<bool>("tFormationDoYawControl",
                                             "Do you want to Yaw controller to be enabled",
                                             false, false, false);

    tLookAtOrigin_wf = addOption<bool>("tLookAtOrigin_wf", "Do you want the Robot to look at the worldframe's origin",
                                       true, false, false);

    tFormationDoYawFixPointControl = addOption<bool>("tFormationDoYawFixPointControl",
                                                     "Do you want to Yaw controller to be enabled",
                                                     false, false, false);


    objectPoint_ = Position3D(0.0,0.0,0.0);

    objectAtOrigin_ = Position3D(0.0,0.0,0.0);


    tFlyToDestination = Position3D(-0.5,-0.5,-2.0);
    tFlyAndLookAtDestination = Position3D(0.0,0.0,-2.0);
    tFlyToVelocity = addOption<double>("tFlyToVelocity",
                                       "Defines the Velocity of the LinearFlyTo Behavior",
                                       1.0, false, false);
    tFlyToDestinationRadius = addOption<double>("tFlyToDestinationRadius",
                                                "Defines the Radius from the Destination, at which the LinearFlyTo Behavior turns invalid",
                                                0.25, false, false);
    tFlyToTopic = addOption<std::string>("tFlyToTopic", "Topic Name for the Fly To Point",
                                         "/flyTo", false, false);


    tMaxYawRate = addOption<double>("tMaxYawRate",
                                    "Max Yaw Rate",
                                    50*(M_PI/180), false, false);


    tUsesHumanInput = addOption<bool>("tUsesHumanInput",
                                      "If the robot should use the human control input",
                                      true, false, false);

    // no Parameters
    //parameterInitialized = true;
}

void Flyto_safe::destroy()
{

}

bool Flyto_safe::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
    // fail of nodeID not known
    if (nodeID < 0) {
        ROS_ERROR("No NodeID set!");
        return false;
    }

    //ROS_INFO("tUsesHumanInput = %i", tUsesHumanInput->getValue());


    // Start with current Position
    virtualPoint = currentState.position;
    tCurrentSelfPosition = currentState.position;
    
        // Also set the first position for the formation to the current state, meaning do not start flying anywhere
    tFlyToDestination = currentState.position;
    tFlyToDestination(2) = -2.0; // fixing the z coordinate

    // add Neighbors
    std::vector<int> neighborVector = tNeighbors->getValue();

    // put to right size
    distanceVector.resize(neighborVector.size());
    formationRepulsiveGradientVector.resize(neighborVector.size());
    //formationAttractiveGradientVector.resize(neighborVector.size());

    //	if (neighborVector.size() != distanceVector.size()) {
    //		ROS_ERROR("Distance and Neighbor Vector not matching!");
    //		return false;
    //	}

    for (unsigned int i = 0; i < neighborVector.size(); i++) {
        addNeighbor(neighborVector[i]);
    }

    lastVelocityInput = Velocity3D::Zero();
    //lastYawRateInput = 0.0;


    // Wait for all neighboring Points.
    bool receivedAll = false;
    Timer timeout;
    while (!receivedAll) {
        receivedAll = true;
        for (unsigned int i = 0; i < neighborVector.size(); i++) {
            if (!neighbors[ neighborVector[i] ]->receivedOnce()) {
                receivedAll = false;
                break;
            }
        }

        if (timeout.getElapsed().toDSec() > 2.0) {
            ROS_ERROR("Could not get neighbor states within timeout!");
            return false;
        }

        // short sleep 10ms
        usleep(10 * 1000);
        publishVP(virtualPoint); // send out own.
    }


    // fill Distance Vector // Create Potential Functions
    for (unsigned int i = 0; i < neighborVector.size(); i++) {
        Position3D neighborVP = neighbors[ neighborVector[i] ]->getVirtualPoint();
        distanceVector[i] = (currentState.position - neighborVP).norm();
        formationRepulsiveGradientVector[i] = new CoTanRepulsiveGradient(
                    "FormationRepulsiveGradient_Neighbor_" + boost::lexical_cast<std::string>(neighborVector[i]),
                    distanceVector[i], distanceVector[i]-0.3, 1.0, 1);
        //formationAttractiveGradientVector[i] = new CoTanAttractiveGradient(
        //            "FormationAttractiveGradient_Neighbor_" + boost::lexical_cast<std::string>(neighborVector[i]),
        //            distanceVector[i], distanceVector[i]+0.3, 1.0, 1);
        //		ROS_ERROR("Distance to (QC %d): %f", neighborVector[i], distanceVector[i]);
    }
    // Init Dead Man
    deadManPressed = false;

    // Subscribe
    obsPointSub = nodeHandle.subscribe(tObsPointsTopicName->getValue(), 1, &Flyto_safe::obsPointCB, this);
    joySub = nodeHandle.subscribe(tJoystickTopic->getValue()
                                  , 10, &Flyto_safe::joystickCB, this);
    if (tVelocityInputEnabled->getValue()) {
        userInputSub = nodeHandle.subscribe(tVelocityInputTopic->getValue(), 1, &Flyto_safe::userVelocityCB, this);
        std::cout << "!!!!!!!!!!!!!!!!!!!!!! tVelocityInputEnabled->getValue()" << std::endl;
    }
    if(tFormationDoYawFixPointControl->getValue()){
        objectPoint_ = Position3D(0.0,0.0,0.0);
        objectPointSub_ = nodeHandle.subscribe(tYawFixPointInputTopic->getValue(), 1, &Flyto_safe::objectPointCB, this);
        ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! subscribing to yaw fix point");
    }
    if(tUsesHumanInput->getValue()){
        ROS_DEBUG_STREAM("subscribing to flyto" << tFlyToTopic->getValue());
        _flytoPoseSubscriber = nodeHandle.subscribe<geometry_msgs::Pose>(tFlyToTopic->getValue(), 1,
                                                                         boost::bind(
                                                                             &Flyto_safe::flytoPoseCallback,                                                                             this,_1));
    }



    timeout.reset();
    // done setting up
    bool allDone = false;
    while (!allDone) {
        publishInitDone();
        allDone = true;
        for (unsigned int i = 0; i < neighborVector.size(); i++) {
            if (!neighbors[ neighborVector[i] ]->initDone()) {
                allDone = false;
                //				break;
            }
        }

        if (timeout.getElapsed().toDSec() > 2.0) {
            ROS_ERROR("Did not receive initDone Msg from all Neighbors!");
            publishInitDone(false);
            return false;
        }

        usleep(1000 * 10);
        publishVP(virtualPoint); // send out own.
        publishInitDone(); // we are done
    }

    tFormationInitialYawAngle = currentState.getEulerRPY()(2);
    // switch into Behavior
    valid = true;

    posModeLastInputTime = Time();


    return true;
}

void Flyto_safe::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{

}

void Flyto_safe::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
    obsPointSub.shutdown();

    joySub.shutdown();
    _flytoPoseSubscriber.shutdown();

    userInputSub.shutdown();
    objectPointSub_.shutdown();

    std::vector<int> neighborVector = tNeighbors->getValue();
    for (unsigned int i = 0; i < neighborVector.size(); i++) {
        removeNeighbor(neighborVector[i]);
        delete formationRepulsiveGradientVector[i];
        //delete formationAttractiveGradientVector[i];
    }
}

void Flyto_safe::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
    // not used
}

void Flyto_safe::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
    generatedTrajInput.setVelocity( Velocity3D::Zero() );
    generatedTrajInput.setYawRate( 0.0 );
}

void Flyto_safe::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
  
    tCurrentSelfPosition = currentState.position;
    
    double timeDiffSec = (Time() - posModeLastInputTime).toDSec();
    posModeLastInputTime = Time();


    Eigen::Vector3d totalVel = Velocity3D::Zero();
    for(double internalTime=0.0;internalTime < timeDiffSec; internalTime += INTERNAL_SUB_STEP){
        double internalTimeStep = fmin(INTERNAL_SUB_STEP,timeDiffSec - internalTime);
        //		printf("internalTimeStep: %f\n", internalTimeStep);

        /*other virtual points*/
        totalVel = getFormationVirtPointVel();
        totalVel += obsPotential.getObstacleVelocity(virtualPoint, lastObstaclePoints);
        /*external commanded velocity*/

        if (tUsesHumanInput->getValue()) {
            //totalVel += lastVelocityInput;
            Vector3D direction = tFlyToDestination - currentState.position;
            if (direction.norm() > tFlyToDestinationRadius->getValue()){
                totalVel += direction.normalized() * tFlyToVelocity->getValue() ;
            }
        }



        virtualPoint += totalVel * internalTimeStep;


    }

    publishVP(virtualPoint);

    generatedTrajInput.setPosition(virtualPoint, totalVel);

    // Calculate Yaw
    double yawRateGain = 10.0;
    double currentYaw = currentState.getEulerRPY()(2);


    //desiredYawRate = fmin(tMaxYawRate->getValue(), desiredYawRate);
    if (tFormationDoYawControl->getValue()) {
        double desiredYawRate = yawRateGain*(Angle::normPi(atan2(lastVelocityInput(1), lastVelocityInput(0))
                                                           - currentYaw));
        if (fabs(desiredYawRate) > tMaxYawRate->getValue()) {
            desiredYawRate = copysign(tMaxYawRate->getValue(), desiredYawRate);
        }
        generatedTrajInput.setYawRate(desiredYawRate);

    }else if(tFormationDoYawFixPointControl->getValue()){ // if the robot should fixiate on one point in the room
        // get the desired yaw rate
        // which is the difference between the orientation towards the yawfixpoint and the current yaw
        // times the yawRategain.

        ROS_INFO_STREAM_THROTTLE(10,"yawFixPoint x: " << tFlyAndLookAtDestination(0) <<  " y: " << tFlyAndLookAtDestination(1));
        double desiredYaw = atan2(tFlyAndLookAtDestination (1) - currentState.position(1), tFlyAndLookAtDestination(0)- currentState.position(0));
        double desiredYawRate = yawRateGain*(Angle::normPi(desiredYaw - currentYaw));
        // check if the yawRate is within max bounds
        if (fabs(desiredYawRate) > tMaxYawRate->getValue()) {
            desiredYawRate = copysign(tMaxYawRate->getValue(), desiredYawRate);
        }
        // when the desiredYawRate falls under this threshold no rotaional velocity will be applied
        if(fabs(desiredYawRate) < 0.05) {
            desiredYawRate = 0.0f;
        }
        // set the desiredYawRate
        generatedTrajInput.setYawRate(desiredYawRate);
    } else {
        generatedTrajInput.setYawAngle(tFormationInitialYawAngle);
    }

}

void Flyto_safe::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
    generatedTrajInput.setVelocity( Velocity3D::Zero() );
    generatedTrajInput.setYawRate( 0.0 );
}

bool Flyto_safe::isValid(const TKState& currentState) const
{
    return valid;
}



Velocity3D Flyto_safe::getFormationVirtPointVel() {
    Velocity3D totalVel(0.0,0.0,0.0);

    //_apVar->otherVPPotential = 0.0;

    std::vector<int> neighborVector = tNeighbors->getValue();

    for(unsigned int i = 0; i < neighborVector.size(); i++) {
        Position3D posDiff = virtualPoint - neighbors[ neighborVector[i] ]->getVirtualPoint();
        double neighborDist = posDiff.norm();
        Position3D posDiffUnit = posDiff.normalized();

        // Repulsive Gradient
	//ROS_INFO("Only repulsive forces are active now");
        totalVel += formationRepulsiveGradientVector[i]->getPotential(neighborDist) * posDiffUnit * tFormationRepulGain->getValue();

        // We switch this off and therefore each robot is controllable separately as they dont stick to each other anymore
	// Attractive Gradient (//beware -)
        //totalVel -= formationAttractiveGradientVector[i]->getPotential(neighborDist) * posDiffUnit;

    }

    return totalVel;

}


}
