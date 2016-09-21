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

#ifndef TELEOP_H
#define TELEOP_H


// ros
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <rosgraph_msgs/Clock.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>

// euro challenge 
#include "mav_msgs/CommandAttitudeThrust.h"



// Telekyb
#include <telekyb_msgs/TKState.h>
#include <telekyb_msgs/Behavior.h>

#include <telekyb_base/Options.hpp>
#include <telekyb_interface/TeleKybCore.hpp>
#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>
#include <telekyb_base/Messages.hpp>
#include <telekyb_base/Spaces.hpp>

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_srvs/BehaviorOutput.h>

// tf
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

#include <eigen3/Eigen/Eigen>

using namespace telekyb;



class FlyTo_Wrapper : telekyb_interface::ActiveBehaviorListener
{
protected:
  // the System
  telekyb_interface::TeleKybCore* _telekybCore;

  // BehaviorController
  telekyb_interface::BehaviorController* _bController;
  // Option Controller
  telekyb_interface::OptionController *_oController;  
  
  // Custom
  telekyb_interface::Behavior _fixedPointHover;

  // Behavior
  telekyb_interface::Behavior* _activeBehaviorPtr;
  
  // System Behaviors
  telekyb_interface::Behavior _ground;
  telekyb_interface::Behavior _hover;
  telekyb_interface::Behavior _normalBreak;
  telekyb_interface::Behavior _takeOff;
  telekyb_interface::Behavior _land;
  telekyb_interface::Behavior _joypadBehavior;  
  telekyb_interface::Behavior _flyto;

  
  
  bool interrupted;  
  
  
  public:
  FlyTo_Wrapper();

  int run();
    // setup
  void setBehaviorCtlr(telekyb_interface::BehaviorController* bController,telekyb_interface::Behavior* aBehaviourPtr);
  void setup();
  void setRobotID(int robotID);
  ~FlyTo_Wrapper();
  
private:
  ros::NodeHandle _node;
  ros::Subscriber _joySubscriber;
  ros::Subscriber _tkStateSubscriber;
  ros::Subscriber _filtflytoPoseSubscriber;

  // Parameter
  std::string _joyTopic;
  std::string _tkStateTopic;
  std::string _flytoPoseTopic;
  
  double _thrustGain;
  double _rollGain;
  double _pitchGain;
  double _yawGain;

  double _ZGain;
  double _YGain;
  double _XGain;
  double _yawDotGain;

  double _maxThrust;
  double _hoverThrust;
  
  // Member
  sensor_msgs::Joy _joyMsg;
  telekyb::TKState _tkState;
  nav_msgs::Path _currentPath;
  bool _currentPathCalledbackOnce;
  int _commandSeq;
  int _opMode; // Mode of operation
  int _robotID;
  bool _joyMsgCalledbackOnce;
  int _joyMsg_lastSeq;
  bool _isSetUp;
  ros::Time _joyMsgLastCall;
  bool _interrupted;
  bool _isInAir;
  geometry_msgs::Pose _flytoPose;
  bool _flyToPose_calledback_once;

  bool _commandHoverActive;
  bool _commandJoyActive;



  

  mav_msgs::CommandAttitudeThrust _outputCommand;
  // callbacks
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void tkStateCallback(const telekyb_msgs::TKState::ConstPtr& state);
  void filtflytoPoseCallback(const geometry_msgs::Pose::ConstPtr& pose);
  
  // logic
  void altitudeCtrl();
  void automatic();
  
    // From Interface
  void activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior); 
  
};
#endif // TELEOP_H
