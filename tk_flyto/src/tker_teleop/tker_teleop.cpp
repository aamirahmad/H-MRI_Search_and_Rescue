/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2014  eugen ruff eugen.ruff@tuebingen.mpg.de
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "tker_teleop.h"
#include <ros/console.h>
#include <sstream>


// Options
ExperimentOptions::ExperimentOptions(): OptionContainer("TKER_COREOPTIONS")
{
    tJoystickTopic = addOption<std::string>("tJoystickTopic",
                                            "JoystickTopic that published sensor_msgs::Joy","/joy", false, true);
    tInterruptTopic = addOption<std::string>("tInterruptTopic",
                                             "interrupt topic to use (std_msgs::Bool)", "/mkinterface_outdoor/humanOperator/interrupt", false, true);
    tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!", false, false, true);
    tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic", "Topic Name of Input of User Velocity",
                                                 "undef", false, false);
    tCommandedYawRateTopic = addOption<std::string>("tCommandedYawRateTopic", "Topic Name of Input of User Velocity",
                                                    "undef", false, false);
    tYawSinComponentTopic = addOption<std::string>("tYawSinComponentTopic", "Topic Name of Input of User Velocity",
                                                   "undef", false, false);
    tSinPulse = addOption("tSinPulse","Pulse for the yaw motion", 1.0, false ,false);
    tSinAmplitude = addOption("tSinAmplitude","Amplitude for the yaw motion", 0.5, false ,false);
    //tTKStateTopic = addOption<std::string>("tTKStateTopic", "Name of the TKState topic","/TeleKyb/TeleKybCore_0/Sensor/TKState", true, true);

}


namespace TKER{

enum commandMapping{
    // axes
    COMMAND_THRUST = 1,
    COMMAND_ROLL = 3,
    COMMAND_PITCH = 4,
    COMMAND_YAW_RATE = 0,
    // buttons
    COMMAND_HOVER = 3, // Logitech Joypad Y
    COMMAND_TAKEOFF = 1, // Logitech Joypad B
    COMMAND_LAND = 0, // Logitech Joypad A
    COMMAND_ACTIVATE_JOY = 2, // Logitech Joypad X
    COMMAND_THRUST_CTRL = 4, // Logitech Joypad LB
    COMMAND_PATHFOLLOW = 5 // Logitech Joypad RB
};

enum ctrlMode{
    THRUST_CTRL = 0,
    ALTITUDE_CTRL = 1,
    AUTOMATIC_CTRL = 2

};

teleop::teleop()
    : _node(telekyb::ROSModule::Instance().getMainNodeHandle()), _telekybCore(NULL)
{
    _commandSeq = 0;
    _joyMsgCalledbackOnce = false;
    _outputCommand.header.frame_id = "base_link";
    _opMode = ALTITUDE_CTRL;


    // get the paramter from the ros parameter server
    _node.param<int>("robotID",_robotID,1);



    std::stringstream default_stateTopic;
    default_stateTopic << "/TeleKyb/TeleKybCore_" << _robotID << "/Sensor/TKState";
    ROS_DEBUG_STREAM("tker::teleop default_stateTopic " << default_stateTopic.str());
    _node.param<std::string>("joyTopic", _joyTopic, "/TeleKyb/tJoy/joy");
    _node.param<std::string>("tkStateTopic", _tkStateTopic, default_stateTopic.str());
    _node.param<double>("thrustGain",_thrustGain,0.01);
    _node.param<double>("rollGain",_rollGain,0.01);
    _node.param<double>("pitchGain",_pitchGain,0.01);
    _node.param<double>("yawGain",_yawGain,0.001);

    _node.param<double>("ZGain",_ZGain,2);
    _node.param<double>("YGain",_YGain,2);
    _node.param<double>("XGain",_XGain,2);
    _node.param<double>("yawDotGain",_yawDotGain,0.1);


    _node.param<double>("maxThrust",_maxThrust,18.0);
    _node.param<double>("hoverThrust",_hoverThrust,15.0);
    _node.param<std::string>("commandAttitudeThrustTopic",_commandAttitudeThrustTopic,"/firefly/command/attitude");
    

    _telekybCore = telekyb_interface::TeleKybCore::getTeleKybCore(_robotID);
    if (!_telekybCore) {
        // fail
        ros::shutdown();
        return;
    }

    
    
    _bController = _telekybCore->getBehaviorController();
    _oController = _telekybCore->getOptionController();


    //activeBehavior = bController->getActiveBehaviorReference();
    _bController->setActiveBehaviorListener(this);

    _activeBehaviorPtr = _bController->getActiveBehaviorPointer();

    


    _joySubscriber = _node.subscribe<sensor_msgs::Joy>(_joyTopic, 1, boost::bind(&teleop::joyCallback,this,_1));
    _tkStateSubscriber = _node.subscribe<telekyb_msgs::TKState>(_tkStateTopic, 1, boost::bind(&teleop::tkStateCallback,this,_1));
    _joyPublisher = _node.advertise<mav_msgs::CommandAttitudeThrust>(_commandAttitudeThrustTopic,1);

    
    
    this->setup();
}


teleop::~teleop()
{

}


void teleop::setup()
{

    // load Behaviors
    _ground = _bController->getSystemBehavior("tk_behavior/Ground");
    _hover = _bController->getSystemBehavior("tk_behavior/Hover");
    _normalBreak = _bController->getSystemBehavior("tk_behavior/NormalBrake");
    _normalBreak.getOptionContainer().getOption("tBrakeInPosition").set(true);
    _takeOff = _bController->getSystemBehavior("tk_behavior/TakeOff");
    _land = _bController->getSystemBehavior("tk_behavior/Land");
    


    // sanity check
    if (_ground.isNull() || _hover.isNull() || _normalBreak.isNull() || _takeOff.isNull() || _land.isNull() ) {
        ROS_FATAL("Unable to get SystemBehavior!!!");
        //ROS_BREAK();
        ros::shutdown();
    }

    // done
    _takeOff.setParameterInitialized(true);

    _land.setParameterInitialized(true);
    _land.setNextBehavior(_ground);
    ROS_INFO("preparing Joystick Behavior");
     // setup joystick
    _joypadBehavior = _bController->loadBehavior("tk_be_common/Joystick");

    _joypadBehavior.getOptionContainer().getOption("tJoystickTopic").set(_joyTopic);
    _joypadBehavior.getOptionContainer().getOption("tJoystickUsePositionMode").set(false);
    _joypadBehavior.getOptionContainer().getOption("tJoystickUseRelativeMode").set(true);
    _joypadBehavior.getOptionContainer().getOption("tJoystickUseDeadManSwitch").set(false);
    _joypadBehavior.getOptionContainer().getOption("tJoystickYawRateScale").set(_yawDotGain);

    _joypadBehavior.getOptionContainer().getOption("tJoystickXScale").set(_XGain);
    _joypadBehavior.getOptionContainer().getOption("tJoystickYScale").set(_YGain);
    _joypadBehavior.getOptionContainer().getOption("tJoystickZScale").set(_ZGain);

    _joypadBehavior.setParameterInitialized(true);

//    // RCJoy behavior to be able to controle to robot like a rc toy
//    _RCJoy = _bController->loadBehavior("tk_be_common/RCJoy");

//    _RCJoy.getOptionContainer().getOption("tRCJoyTopic").set(_joyTopic);
//    _RCJoy.getOptionContainer().getOption("tRCJoyUseDeadManSwitch").set(false);
//    _RCJoy.getOptionContainer().getOption("tRCJoyYawRateScale").set(_yawDotGain);

//    _RCJoy.setParameterInitialized(true);

    // setup the path following behavior

//    _mpcPathFollower = _bController->loadBehavior("tk_be_common/mpcPathFollow");
//    _mpcPathFollower.getOptionContainer().getOption("tmpcPathFollowPathTopic").set("/path/Path");

//    _PathFollower = _bController->loadBehavior("tk_be_common/PathFollow");
//    _PathFollower.getOptionContainer().getOption("tPathFollowPathTopic").set("/path/Path");


    _fixedPointHover = _bController->loadBehavior("tk_be_common/FixedPointHover");

    _fixedPointHover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(telekyb::Position3D(0.0,0.0,-1.0));

    _fixedPointHover.setParameterInitialized(true);


    _takeOff.getOptionContainer().getOption("tTakeOffDestination").set(telekyb::Position3D(0.0,0.0,-1.0));
    // takeoff in current position
    _takeOff.getOptionContainer().getOption("tTakeOffInPosition").set(true);


    if (*_activeBehaviorPtr != _ground) {
        ROS_ERROR("UAV not in Ground Behavior during Startup");
        ros::shutdown();
    }

    // lastly start Controller


    _interrupted = false;

    _commandHoverActive = false;
    _commandJoyActive = false;
    _currentPathCalledbackOnce = false;
}
void teleop::tkStateCallback(const telekyb_msgs::TKState::ConstPtr& state)
{
    _tkState.fromTKStateMsg(*state);
}


void teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    _joyMsg = *msg;
    _joyMsgCalledbackOnce = true;
    _joyMsgLastCall = ros::Time::now();
}

void teleop::pathCallback(const nav_msgs::PathConstPtr &path){
    ROS_WARN_STREAM("recieved new path");
    _currentPath = *path;
    _currentPathCalledbackOnce = true;
}


int teleop::findClosestWayPoint(TKState &state, const nav_msgs::Path &path){

    double dist = INFINITY;
    unsigned int currentIdx = 0;
    //    ROS_WARN_STREAM("_k " << _k);
    for(size_t k = 0; k < path.poses.size(); k++){
        double tmpDist = sqrt(pow(state.position(0)-path.poses[k].pose.position.x,2)
                +pow(state.position(1)-path.poses[k].pose.position.y,2)
                +pow(state.position(2)-path.poses[k].pose.position.z,2));
        //ROS_WARN_STREAM("tmp dist " << tmpDist << " dist " << dist << " k " << _k);
        if(dist > tmpDist){
            dist = tmpDist;
            currentIdx = k;
        }
    }
    return currentIdx;

}

void teleop::altitudeCtrl()
{

    ROS_DEBUG_STREAM_THROTTLE(2,"tkState position x " <<  _tkState.position(0) <<" y " <<  _tkState.position(1) << " z " << _tkState.position(2));
    double joyMsgSum = 0.0;

    for(size_t i = 0; i < _joyMsg.axes.size();i++){
        joyMsgSum += fabs(_joyMsg.axes[i]);
    }

    // if the command hover button is pressed the hover mode becomes active and the Joystick mode inactive
    if(_joyMsg.buttons[COMMAND_HOVER] > 0){
        _commandHoverActive = true;
        _commandJoyActive = false;

    }
    // as before but with the COMMAND_ACTIVATE_JOY button
    if(_joyMsg.buttons[COMMAND_ACTIVATE_JOY] > 0){
        _commandHoverActive = false;
        _commandJoyActive = true;
    }
    // the behavior is switched to hover if the hovermode is active
    // or the last joystick callback is outside the timeout
    if(*_activeBehaviorPtr != _fixedPointHover){
        if(_commandHoverActive || (_commandJoyActive &&  joyMsgSum == 0.0 && _joyMsgLastCall + ros::Duration(4.0) < ros::Time::now()) ){
            _fixedPointHover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(telekyb::Position3D(_tkState.position(0),_tkState.position(1),_tkState.position(2)));
            _fixedPointHover.getOptionContainer().getOption("tFixedPointHoverHoveringOrientation").set(_tkState.getEulerRPY()(2));
            _bController->switchBehavior(_fixedPointHover);
        }
    }
    // the behvior switch to the teleop behavior is done and only done
    // if the joy mode is active and the last joy callback is within the timeout

    // don't swtich to an active behavior
    if(*_activeBehaviorPtr != _joypadBehavior){
        if(_commandJoyActive && joyMsgSum > 0.0){
            _bController->switchBehavior(_joypadBehavior);
        }
    }
    if(_joyMsg.buttons[COMMAND_TAKEOFF] > 0){

        _takeOff.getOptionContainer().getOption("tTakeOffDestination").set(telekyb::Position3D(_tkState.position(0),_tkState.position(1),-1.0));
        if(*_activeBehaviorPtr != _takeOff)
            _bController->switchBehavior(_takeOff);
        _commandHoverActive = false;
        _commandJoyActive = false;
    }
    if(_joyMsg.buttons[COMMAND_LAND] > 0){
        if(*_activeBehaviorPtr != _land)
            _bController->switchBehavior(_land);
        _commandHoverActive = false;
        _commandJoyActive = false;
    }

//    if(_joyMsg.buttons[COMMAND_THRUST_CTRL] > 0){
//        if(*_activeBehaviorPtr != _RCJoy){
//            _bController->switchBehavior(_RCJoy);
//            ROS_WARN("SWITCH TO RCJoy MODE");
//        }
//        _opMode = THRUST_CTRL;
//        _commandHoverActive = false;
//        _commandJoyActive = false;
//    }

    if(_joyMsg.buttons[COMMAND_PATHFOLLOW] > 0){
        if(*_activeBehaviorPtr != _PathFollower){
            _bController->switchBehavior(_PathFollower);
            ROS_WARN("SWITCH TO PATHFOLLOW MODE");
        }
        _opMode = AUTOMATIC_CTRL;
        _commandHoverActive = false;
        _commandJoyActive = false;
    }

}
void teleop::thrustCtrl()
{
    _outputCommand.thrust += _joyMsg.axes[COMMAND_THRUST]*_thrustGain;
    if(_outputCommand.thrust < 0){
        _outputCommand.thrust = 0;
    }
    if(_outputCommand.thrust > _maxThrust){
        _outputCommand.thrust = _maxThrust;
    }
    if(_joyMsg.buttons[COMMAND_LAND] > 0){
        _opMode = ALTITUDE_CTRL;
    }

}


void teleop::automatic(){
    // watchdog
    // if any button except the COMMAND_AUTOMATIC button is pressed
    // the behavior is switched back to ALTITUDE_CTRL
    for(size_t i = 0; i < _joyMsg.buttons.size(); i++){
        if(_joyMsg.buttons[i] != 0 && i != COMMAND_THRUST_CTRL && i != COMMAND_PATHFOLLOW) {
            ROS_WARN("BUTTON TOUCHED: SWITCH TO ALTITUDE_CTRL MODE");
            _opMode = ALTITUDE_CTRL;
            _commandHoverActive = true;
        }
    }
    // if any axes is triggered the behavior is swtiched back
    // to ALTITUDE_CTRL
    // except for LT and RT because they might be one or zero
    for(size_t i = 0; i < _joyMsg.axes.size(); i++){
        if(_joyMsg.axes[i] != 0 && !(i == 2 || i == 5)){
            ROS_WARN("AXES TOUCHED: SWITCH TO ALTITUDE_CTRL MODE");
            _opMode = ALTITUDE_CTRL;
            _commandHoverActive = true;
        }
    }

}

void teleop::activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior)
{

}

int teleop::run(){

    /*
     *  set logger
    */
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::Rate loopRate(60);
    ROS_INFO_STREAM("ros node name " << _node.getNamespace());


    int i = 0;
    int activeRobot = -1;
    while(ros::ok()){
        ros::spinOnce();
        //_node.getParam("/activeRobot",activeRobot);

        if(_joyMsgCalledbackOnce){

            _outputCommand.header.seq = _commandSeq++;
            _outputCommand.header.stamp = ros::Time::now();

            switch(_opMode){
            case(THRUST_CTRL):
                this->thrustCtrl();
                _outputCommand.pitch = _joyMsg.axes[COMMAND_PITCH]*_pitchGain;
                _outputCommand.roll = -1*_joyMsg.axes[COMMAND_ROLL]*_rollGain;
                _outputCommand.yaw_rate = _joyMsg.axes[COMMAND_YAW_RATE]*_yawGain;
                _joyPublisher.publish<mav_msgs::CommandAttitudeThrust>(_outputCommand);
                break;
            case(ALTITUDE_CTRL):
                this->altitudeCtrl();
                break;
            case(AUTOMATIC_CTRL):
                this->automatic();
                break;
            }
        }

        ROS_DEBUG_STREAM_THROTTLE(5,"tker_teleop ros::Rate cycleTime " << loopRate.cycleTime() );
        loopRate.sleep();
    }
    return 0;



}

}// endof namespace

