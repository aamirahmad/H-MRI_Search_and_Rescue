
#include <cstdio>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sys/time.h> 

//#define TERMINAL_DEBUG
#undef TERMINAL_DEBUG

//#define DRAW_DEBUG
#undef DRAW_DEBUG

using namespace std;
using namespace ros;

class tf_closer
{
  
    double meanFactor;
    double devFactor;
    
    bool detectionSucces_; // /*!< true if detector detects the object in the image */
    
    string baseImageTopic;
    string maskImageTopic;
    string maskedImageTopic;
    string camInfoTopic;
    string robotPoseTopic;
    string projectedObjectLink;
    
    string camBaseLink;
    string camRGBFrameLink;
    string camRGBOpticalFrameLink;
    string detectedObjectLink;
    string robotBaseLink;
    string detectedObject_Wframe_Topic;
    string targetBlobTopic;
    
    NodeHandle nh_;
    Subscriber camInfoSub_,robotPoseSub_,objectGTPoseSub_;
    
    int minAreaOfTargetProject;
    double objectRealSurfaceArea;
    double Z_obj_camfrm;
    double X_obj_camfrm;
    double Y_obj_camfrm;
     ///@hack for February demo. @Fix this
    double camOptCenterWorld[3];
    double objWorldFrame[3];
    double projObjWorldFrame[3];
      
    //camera parameters
    double camFx, camFy;
    double camCx, camCy;
    
    //tf broadcaster and transforms
    tf::TransformBroadcaster br;
    tf::Transform tfObCam,tfCamRob,tfRobWorld;//Object in cam frame,Cam in robot frame, Robot in World frame
    tf::TransformListener listenerObjWorld;
    tf::StampedTransform transformObjWorld;
    
    tf::TransformListener listenerCamWorld;
    tf::StampedTransform transformCamWorld;
    
    tf::TransformListener listenerprojObjCam;
    tf::StampedTransform transformprojObjCam;    
    
    //detected object pose in world frame
    geometry_msgs::PoseWithCovarianceStamped poseObjWorld;
    // variables for transforming covariance
    geometry_msgs::Pose poseObjectRob_;
    geometry_msgs::Pose poseRobWorld_;
    geometry_msgs::PoseWithCovariance poseObjWorld_;
    geometry_msgs::PoseWithCovarianceStamped poseObjWorldStamped_;
    geometry_msgs::Pose poseOpticalframeWorld_;
    geometry_msgs::PoseWithCovariance poseObjOpticalframe_;  
    
    
    int robotID;
    
  
  public:
    tf_closer(NodeHandle &_nh, int _robotID): nh_(_nh), robotID(_robotID)
    {
  
      nh_.getParam("robotPoseTopic", robotPoseTopic);
  
       // Other subscribers
      robotPoseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/TeleKyb/Vicon/Quadcopter_"+boost::lexical_cast<string>(robotID)+"/Quadcopter_"+boost::lexical_cast<string>(robotID), 10,boost::bind(&tf_closer::storeLatestRobotPose,this,_1));

      
       //Setting up fixed transformations if any
      tfCamRob.setOrigin( tf::Vector3(0.0975,0.0,-0.04603));
      tf::Quaternion q_1(0.0,0.38,0.0,0.924);
      tfCamRob.setRotation(q_1);
      

    }
    

    /*! \brief This is a method for reading recent robot pose and storing them into private variables
    * of the class
    */
    void storeLatestRobotPose(const geometry_msgs::PoseStamped::ConstPtr&);    

    
};