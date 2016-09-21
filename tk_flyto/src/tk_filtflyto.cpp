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


#include <telekyb_base/TeleKyb.hpp>
#include <ros/ros.h>

#include "tk_filtflyto.hpp"
#include <ros/console.h>
#include <sstream>


enum commandMapping{
    // buttons
    COMMAND_HOVER = 3, // Logitech Joypad Y
    COMMAND_TOGGLE_LIFTLAND = 1, // Logitech Joypad B
    COMMAND_LAND = 0, // Logitech Joypad A
    COMMAND_ACTIVATE_JOY = 2, // Logitech Joypad X
    COMMAND_FILTFLYTO = 5, // Logitech Joypad RB
};

enum ctrlMode{
    ALTITUDE_CTRL = 1,
    AUTOMATIC_CTRL = 2

};

FlyTo_Wrapper::FlyTo_Wrapper()
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
    std::stringstream default_flytoTopic;
    default_flytoTopic << "/filtFlyTo_" << _robotID;


    ROS_DEBUG_STREAM("tker::tk_flyto default_stateTopic " << default_stateTopic.str());
    _node.param<std::string>("joyTopic", _joyTopic, "/TeleKyb/tJoy/joy");
    _node.param<std::string>("tkStateTopic", _tkStateTopic, default_stateTopic.str());
    _node.param<std::string>("flytoTopic", _flytoPoseTopic, default_flytoTopic.str());


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


    _joySubscriber = _node.subscribe<sensor_msgs::Joy>(_joyTopic, 1, boost::bind(&FlyTo_Wrapper::joyCallback,this,_1));
    _tkStateSubscriber = _node.subscribe<telekyb_msgs::TKState>(_tkStateTopic, 1, boost::bind(&FlyTo_Wrapper::tkStateCallback,this,_1));
    _filtflytoPoseSubscriber = _node.subscribe<geometry_msgs::Pose>(_flytoPoseTopic, 1, boost::bind(&FlyTo_Wrapper::filtflytoPoseCallback,this,_1));


    _isSetUp = false;
    this->setup();
    _isSetUp = true;
}


FlyTo_Wrapper::~FlyTo_Wrapper()
{

}


void FlyTo_Wrapper::setup()
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

    _flyto = _bController->loadBehavior("tk_be_common/LinearFlyTo");
    _flyto.getOptionContainer().getOption("tFlyToDestination").set(telekyb::Position3D(0.0,0.0,-1.0));
    _flyto.getOptionContainer().getOption("tFlyToDestinationRadius").set(0.5);

    _flyto.setParameterInitialized(true);

    _fixedPointHover = _bController->loadBehavior("tk_be_common/FixedPointHover");
    _fixedPointHover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(telekyb::Position3D(0.0,0.0,-1.0));
    _fixedPointHover.setParameterInitialized(true);


    _takeOff.getOptionContainer().getOption("tTakeOffDestination").set(telekyb::Position3D(0.0,0.0,-1.0));
    // takeoff in current position
    _takeOff.getOptionContainer().getOption("tTakeOffInPosition").set(true);

    _isInAir = false;
    _flyToPose_calledback_once = false;
    if (*_activeBehaviorPtr != _ground) {
        ROS_ERROR("UAV not in Ground Behavior during Startup");
        ros::shutdown();
    }

    // lastly start Controller
    _joyMsg_lastSeq = -1;

    _interrupted = false;

    _commandHoverActive = false;
    _commandJoyActive = false;
    _currentPathCalledbackOnce = false;
}
void FlyTo_Wrapper::tkStateCallback(const telekyb_msgs::TKState::ConstPtr& state)
{
    _tkState.fromTKStateMsg(*state);
    if(!_flyToPose_calledback_once){
        _flytoPose.position.x = _tkState.position(0);
        _flytoPose.position.y = _tkState.position(1);
        _flytoPose.position.z = _tkState.position(2);
    }
}


void FlyTo_Wrapper::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    _joyMsg = *msg;
    _joyMsgCalledbackOnce = true;
    _joyMsgLastCall = ros::Time::now();
}



void FlyTo_Wrapper::filtflytoPoseCallback(const geometry_msgs::Pose::ConstPtr &pose){
    // setting the next fly to position
    _flytoPose = *pose;

    if(_isSetUp){
        _flyto.getOptionContainer().getOption("tFlyToDestination").set(telekyb::Position3D(pose->position.x,
                                                                                           pose->position.y,
                                                                                           pose->position.z));
        if(_opMode == AUTOMATIC_CTRL){
            if(*_activeBehaviorPtr != _flyto){
                _bController->switchBehavior(_flyto);
            }
        }
    }
    if(!_flyToPose_calledback_once)
        _flyToPose_calledback_once = true;
}
void FlyTo_Wrapper::altitudeCtrl()
{

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
    // the behvior switch to the tk_flyto behavior is done and only done
    // if the joy mode is active and the last joy callback is within the timeout

    // don't swtich to an active behavior
    if(*_activeBehaviorPtr != _joypadBehavior){
        if(_commandJoyActive && joyMsgSum > 0.0){
            _bController->switchBehavior(_joypadBehavior);
        }
    }
    if(_joyMsg.buttons[COMMAND_TOGGLE_LIFTLAND]  > 0 && _joyMsg.header.seq > _joyMsg_lastSeq){
        ROS_INFO_COND(_isInAir,"is in Air");
        if(_isInAir){
            ROS_INFO("landing");
            if(*_activeBehaviorPtr != _land)
                _bController->switchBehavior(_land);
            _commandHoverActive = false;
            _commandJoyActive = false;
            _isInAir = false;

        }else{
            ROS_INFO("takeoff");
            if(*_activeBehaviorPtr != _takeOff){
                _takeOff.getOptionContainer().getOption("tTakeOffDestination").set(telekyb::Position3D(_tkState.position(0),_tkState.position(1),-2.0));
                _bController->switchBehavior(_takeOff);
            }
            _commandHoverActive = false;
            _commandJoyActive = false;
            _isInAir = true;
        }
    }
    if(_joyMsg.buttons[COMMAND_LAND] > 0){
        if(*_activeBehaviorPtr != _land)
            _bController->switchBehavior(_land);
        _commandHoverActive = false;
        _commandJoyActive = false;
    }

    if(_joyMsg.buttons[COMMAND_FILTFLYTO] > 0){

        _flyto.getOptionContainer().getOption("tFlyToDestination").set(telekyb::Position3D(_flytoPose.position.x,
                                                                                           _flytoPose.position.y,
                                                                                           _flytoPose.position.z));

        if(*_activeBehaviorPtr != _flyto){
            _bController->switchBehavior(_flyto);
            ROS_WARN("SWITCH TO FILT FLY TO MODE");
        }
        _opMode = AUTOMATIC_CTRL;
        _commandHoverActive = false;
        _commandJoyActive = false;
    }

    _joyMsg_lastSeq = _joyMsg.header.seq;


}


void FlyTo_Wrapper::automatic(){
    // watchdog
    // if any button except the COMMAND_AUTOMATIC button is pressed
    // the behavior is switched back to ALTITUDE_CTRL
    for(size_t i = 0; i < _joyMsg.buttons.size(); i++){
        if(_joyMsg.buttons[i] != 0 && i != COMMAND_FILTFLYTO) {
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

void FlyTo_Wrapper::activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior)
{

}

int FlyTo_Wrapper::run(){

    /*
     *  set logger
    */
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::Rate loopRate(60);
    ROS_INFO_STREAM("ros node name " << _node.getNamespace());

    while(ros::ok()){
        ros::spinOnce();
        //_node.getParam("/activeRobot",activeRobot);

        if(_joyMsgCalledbackOnce){
            ROS_INFO_STREAM_THROTTLE(10,"Active Behavior " << _activeBehaviorPtr->getBehaviorName());
            _outputCommand.header.seq = _commandSeq++;
            _outputCommand.header.stamp = ros::Time::now();

            switch(_opMode){
            case(ALTITUDE_CTRL):
                this->altitudeCtrl();
                break;
            case(AUTOMATIC_CTRL):
                this->automatic();
                break;
            }
        }

        ROS_DEBUG_STREAM_THROTTLE(5,"tker_tk_flyto ros::Rate cycleTime " << loopRate.cycleTime() );
        loopRate.sleep();
    }
    return 0;



}


int main(int argc, char** argv){


    // Initialize ROS
    //ros::init (argc, argv, "tk_eruff");
    telekyb::TeleKyb::init(argc,argv, "tk_flyto");
    FlyTo_Wrapper* tk_flyto_ptr = new FlyTo_Wrapper();

    tk_flyto_ptr->run();

    delete tk_flyto_ptr;
    telekyb::TeleKyb::shutdown();
    return 0;


}
