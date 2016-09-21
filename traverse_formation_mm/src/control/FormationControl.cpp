/*
 * FormationControl.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#include "FormationControl.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>

#include <string>
#include <iostream>
#include <cstdio>
#include <memory>
#include <typeinfo>
static std::string exec(const char* cmd) {
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}

template<typename T>
void callService(ros::NodeHandle nh,std::string topic,T request ){
    ros::ServiceClient client = nh.serviceClient< T >(topic);
    if (client.call(request))
    {
        ROS_INFO("Success");
    }else{
        ROS_ERROR_STREAM("Failed to call service " << topic);
    }
}

template<typename T>
static void callServiceByName(ros::NodeHandle n,std::string srvName,T &srvRequest){
    std::stringstream cmd;
    cmd << "rosservice list | grep -i " << srvName;
    std::string takeOffDestiationSrvTopics(exec(cmd.str().c_str()));
    std::stringstream ss(takeOffDestiationSrvTopics);
    std::string srvTopic;

    while(std::getline(ss,srvTopic,'\n')){
        std::cout << "try to call service: " << srvTopic << std::endl;
        ros::ServiceClient client = n.serviceClient< T >(srvTopic);

        if (client.call(srvRequest))
        {
            ROS_INFO("Success");

        }else{
            ROS_ERROR_STREAM("Failed to call service " << srvTopic << " " << srvRequest.request);
        }
    }
}
template<typename T>
static bool setService(ros::NodeHandle n,int coreID,std::string topic,T &request){
    std::stringstream srvTopic;
    srvTopic << "/TeleKyb/TeleKybCore_" << coreID;
    srvTopic << topic;
    callServiceByName(n,srvTopic.str(),request);

}
template<typename T>
static T getService(ros::NodeHandle n,int coreID,std::string topic,T request){
    std::stringstream srvTopic;
    srvTopic << "/TeleKyb/TeleKybCore_" << coreID;
    srvTopic << topic;
    callServiceByName(n,srvTopic.str(),request);
    return request;
}

// Options
FormationControlOptions::FormationControlOptions()
    : OptionContainer("FormationControlOptions")
{
    tRobotIDs = addOption< std::vector<int> >("tRobotIDs",
                                              "Specifies the ID of the Robots to connect to", std::vector<int>() , true, true);
    tJoystickTopic = addOption<std::string>("tJoystickTopic",
                                            "Joysticktopic to use (sensor_msgs::Joy)", "/TeleKyb/tJoy/joy", false, true);
    tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic",
                                                 "Velocity Input Topic to use (geometry_msgs::Vector3Stamped)", "FormationVelocityInput", false, true);
    tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!", false, false, true);
    tUsesHumanInput = addOption< std::vector<int> >("tUsesHumanInput",
                                                    "If the robot should use the human control input", std::vector<int>(), true, true);

    tDoYawControl = addOption<bool>("tDoYawControl", "Set to true if you want to ctrl yawRate", false, false, false);
    tDoYawFixPointControl = addOption<bool>("tDoYawFixPointControl",
                                            "Set true if you want the robot to fix on one point", false, false, false);

    tFixPointInputTopic = addOption<std::string>("tFixPointInputTopic",
                                                 "Topic for the fixpoint", "/fixPoint", false, false);

    tMaxYawRate = addOption<double>("tMaxYawRate","Max Yaw Rate",50*(M_PI/180), false, false);


    tFlyToVelocity = addOption<double>("tFlyToVelocity",
                                       "Defines the Velocity of the LinearFlyTo Behavior",
                                       1.4, false, false);
    tFlyToDestinationRadius = addOption<double>("tFlyToDestinationRadius",
                                                "Defines the Radius from the Destination, at which the LinearFlyTo Behavior turns invalid",
                                                0.25, false, false);
    tFlyToTopic = addOption<std::string>("tFlyToTopic", "Topic Name for the Fly To Point",
                                         "/flyTo", false, false);
    
    tUseJoystick  = addOption<bool>("tUseJoystick",
			"If true, uses joystick to start motors, takeoff, start trajectory and land, if false, uses time.", true, false, false);    
}

FormationControl::FormationControl()
    : mainNodeHandle( ROSModule::Instance().getMainNodeHandle() )
{
    setupFormationControl();
}

FormationControl::~FormationControl()
{
    for (unsigned int i = 0; i < formationElements.size(); i++) {
        delete formationElements[i];
    }
}

void FormationControl::setupFormationControl()
{
    // Initialize variables for auto-start of the experiment based on a single external take-off signal
    takeOffSignalReceived = false;
    takeOffDone = false;
    timeAtTakeOff = 1000000.0; //set to very high before take off
    switchedToFormation = false;

    // create elements   
    _robotIDs = options.tRobotIDs->getValue();
    std::vector<int> usesHumanInputInt = options.tUsesHumanInput->getValue();


    if (_robotIDs.size() != usesHumanInputInt.size()) {
        ROS_ERROR("sizes of tRobotIDs and tUsesHumanInput don't match!");
        ros::shutdown();
    }


    Eigen::Vector3d center(0,0,-0.5);
    //	double halfEdgeLength = 1.0;


    for (std::vector<int>::iterator it = _robotIDs.begin(); it != _robotIDs.end();it++) {

        telekyb_srvs::StringInput stringSrv;
        stringSrv.request.input = "[0.0,0.0,-1.0]";

        setService(mainNodeHandle,*it,"/Option/tk_behavior/TakeOff/[0-9]*/tTakeOffDestination/set",stringSrv);

        // convert the robotIDs into a string object which is then send over the service
        std::stringstream robotIDs_ss;
        robotIDs_ss << "[ ";
        for (std::vector<int>::iterator jt = _robotIDs.begin(); jt != _robotIDs.end();jt++) {
            // don't place the robot as neighbour of itself
            if(it != jt){
                robotIDs_ss << *jt << ",";
            }
        }
        robotIDs_ss << "] ";

        stringSrv.request.input = robotIDs_ss.str().c_str();
        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tNeighbors/set",stringSrv);

        stringSrv.request.input = "true";
        setService(mainNodeHandle,*it,"/Option/tk_behavior/TakeOff/[0-9]*/tTakeOffVertically/set",stringSrv);

        stringSrv.request.input = options.tVelocityInputTopic->getValue();
        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tVelocityInputTopic/set",stringSrv);

        stringSrv.request.input = options.tJoystickTopic->getValue();
        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tJoystickTopic/set",stringSrv);

        stringSrv.request.input = "false";
        if(options.tDoYawFixPointControl->getValue()){
            stringSrv.request.input = "true";
        }
        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tFormationDoYawFixPointControl/set",stringSrv);


        std::stringstream objectPoint_ss;
        objectPoint_ss << options.tFixPointInputTopic->getValue();// << *it;
        stringSrv.request.input = objectPoint_ss.str().c_str();
        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tYawFixPointInputTopic/set",stringSrv);

        stringSrv.request.input =  boost::lexical_cast<std::string>(options.tMaxYawRate->getValue());
        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tMaxYawRate/set",stringSrv);


        stringSrv.request.input =  "false";
        if((bool)usesHumanInputInt[ std::distance(_robotIDs.begin(), it)]){
            stringSrv.request.input =  "true";
        }
        ROS_INFO_STREAM("useHumanInput " << usesHumanInputInt[ std::distance(_robotIDs.begin(), it)] << " " << stringSrv.request.input);

        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tUsesHumanInput/set",stringSrv);

        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tVelocityInputEnabled/set",stringSrv);

        stringSrv.request.input = boost::lexical_cast<std::string>(options.tFlyToDestinationRadius->getValue());
        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tFlyToDestinationRadius/set",stringSrv);
        stringSrv.request.input = boost::lexical_cast<std::string>(options.tFlyToVelocity->getValue());
        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tFlyToVelocity/set",stringSrv);


        std::stringstream default_flytoTopic;
        default_flytoTopic << "/filtFlyTo_" << *it;

        stringSrv.request.input = default_flytoTopic.str().c_str();
        setService(mainNodeHandle,*it,"/Option/traverse_formation_mm/Traverse_Formation_mm/[0-9]*/tFlyToTopic/set",stringSrv);


        telekyb_srvs::BoolInput boolSrv;
        boolSrv.request.input = true;
        setService(mainNodeHandle,*it,"/Behavior/[0-9]*/SetParamInit",boolSrv);


    }

    //	for (unsigned int i = 0; i < formationElements.size()-1; i++) {
    //		std::vector<int> neighborVector = robotIDs;
    //		neighborVector.erase(neighborVector.size()-1);
    //		std::vector<double> neighborDistanceVector(formationElements.size()-1,2.0);
    //
    //	}


    // lastly start Controller
    
    // lastly start Controller in joystick CB if using joystick, using time otherwise
    if (options.tUseJoystick->getValue()){
	    joySub = mainNodeHandle.subscribe(options.tJoystickTopic->getValue(), 1, &FormationControl::joystickCB, this);
    }
    else{	    
	    clientTakeoffTrigger = mainNodeHandle.advertiseService("startsignalvr", &FormationControl::triggerTakeOff,this);
	    
            ROS_INFO("Not using joystick... so using timed startup");
	    timeSub = mainNodeHandle.createTimer(ros::Duration(0.05), &FormationControl::timeCB, this);
    }    
}


bool FormationControl::triggerTakeOff(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    takeOffSignalReceived = true;
    ROS_INFO("Take off signal Received");
    return true;
}

void FormationControl::timeCB(const ros::TimerEvent&)
{

	double now = ros::Time::now().toSec();

	std_srvs::Trigger trigger_srv;
	
	
	if (takeOffSignalReceived && !takeOffDone) 
	{
	  for (std::vector<int>::iterator it = _robotIDs.begin(); it != _robotIDs.end();it++) 
	  {
	      std::stringstream srvTopic_ss;
	      srvTopic_ss << "/TeleKyb/formation_slave_" <<  *it << "/LiftLand";
	      ROS_INFO_STREAM(srvTopic_ss.str());
	      callService(mainNodeHandle,srvTopic_ss.str(),trigger_srv);
	  }
	  
	  takeOffDone = true;
	  timeAtTakeOff = ros::Time::now().toSec();
	  ROS_INFO("Take off done at = %f seconds",timeAtTakeOff);
	}

        //first condition waits 10 seconds after take off and then switches to the formation behavior
	if (now-timeAtTakeOff>10.0 && takeOffDone && !switchedToFormation) 
	{
	  ROS_INFO("Switching to formation at = %f seconds",now);
	  for (std::vector<int>::iterator it = _robotIDs.begin(); it != _robotIDs.end();it++) {
	      std::stringstream srvTopic_ss;
	      srvTopic_ss << "/TeleKyb/formation_slave_" <<  *it << "/SwtichToFormation";
	      ROS_INFO_STREAM(srvTopic_ss.str());
	      callService(mainNodeHandle,srvTopic_ss.str(),trigger_srv);
	  }
	  switchedToFormation = true;
	}
}

void FormationControl::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
    // use button 2
    if (msg->buttons.size() < 9) {
        ROS_ERROR("Joytick does not publish enough buttons.");
        return;
    }
    std_srvs::Trigger trigger_srv;
    // Emergency
    if (msg->buttons[5]) { //RB
        for (std::vector<int>::iterator it = _robotIDs.begin(); it != _robotIDs.end();it++) {
            std::stringstream srvTopic_ss;
            srvTopic_ss << "/TeleKyb/formation_slave_" <<  *it << "/Emergency";
            ROS_WARN_STREAM(srvTopic_ss.str());
            callService(mainNodeHandle,srvTopic_ss.str(),trigger_srv);
        }
    }

    if (msg->buttons[0]) { // LB
        for (std::vector<int>::iterator it = _robotIDs.begin(); it != _robotIDs.end();it++) {

            // get the current behavior
            telekyb_srvs::BehaviorOutput behaviorRequest;
            behaviorRequest = getService(mainNodeHandle,*it,"/Behavior/GetActiveBehavior",behaviorRequest);
            // only if the current behavior is Ground and only if its Ground toggle the motors
            if(behaviorRequest.response.behaviorName == "tk_behavior/Ground"){
                std::stringstream srvTopic_ss;
                srvTopic_ss << "/TeleKyb/formation_slave_" <<  *it << "/ToggleMotors";
                ROS_INFO_STREAM(srvTopic_ss.str());
                callService(mainNodeHandle,srvTopic_ss.str(),trigger_srv);
            }
        }
    }
    // LiftLand
    if (msg->buttons[2]) { // X
        for (std::vector<int>::iterator it = _robotIDs.begin(); it != _robotIDs.end();it++) {
            std::stringstream srvTopic_ss;
            srvTopic_ss << "/TeleKyb/formation_slave_" <<  *it << "/LiftLand";
            ROS_INFO_STREAM(srvTopic_ss.str());
            callService(mainNodeHandle,srvTopic_ss.str(),trigger_srv);
        }
    }

    if (msg->buttons[3]) { // Y
        for (std::vector<int>::iterator it = _robotIDs.begin(); it != _robotIDs.end();it++) {
            std::stringstream srvTopic_ss;
            srvTopic_ss << "/TeleKyb/formation_slave_" <<  *it << "/SwtichToFormation";
            ROS_INFO_STREAM(srvTopic_ss.str());
            callService(mainNodeHandle,srvTopic_ss.str(),trigger_srv);
        }
    }

    if (msg->buttons[4]) { // A
        for (std::vector<int>::iterator it = _robotIDs.begin(); it != _robotIDs.end();it++) {
            std::stringstream srvTopic_ss;
            srvTopic_ss << "/TeleKyb/formation_slave_" <<  *it << "/ToggleYaw";
            ROS_INFO_STREAM(srvTopic_ss.str());
            callService(mainNodeHandle,srvTopic_ss.str(),trigger_srv);
        }
    }


}

