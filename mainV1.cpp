//Program from aquiring skeleton joint data from kinect/recording (.oni)
//and dispaying in cloud viewer

#include <XnCppWrapper.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <iostream>
using namespace xn;

#define MAX_DEPTH 10000
float g_pDepthHist[MAX_DEPTH];
#define CHECK_RC(nRetVal, what) 
#define POSE_TO_USE "Psi"
xn::UserGenerator g_UserGenerator;
xn::DepthGenerator g_DepthGenerator;
xn::DepthGenerator depth;


bool CopyDepthRawBufToCvMat16u ( const XnDepthPixel * srcDepthData, cv::Mat & destDepthMat )
{
if (srcDepthData == NULL || destDepthMat.empty()) {
return false;
}
assert ( destDepthMat.type() == CV_16UC1 );
const uchar * destRowPtr = NULL;
XnDepthPixel * destDataPtr = NULL;
const int destHeight = destDepthMat.size().height;
const int destWidth = destDepthMat.size().width;
const int destRowStep = destDepthMat.step;
destRowPtr = destDepthMat.data;
for (int y = 0; y < destHeight; ++y, destRowPtr += destRowStep)
{
destDataPtr = (XnDepthPixel *)(destRowPtr);
for (int x = 0; x < destWidth; ++x, ++srcDepthData, ++destDataPtr)
{
*destDataPtr = *srcDepthData;
}
}
return true;
}




void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
printf("New User: %d\n", nId);
g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(POSE_TO_USE,nId);
}
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
}
void XN_CALLBACK_TYPE Pose_Detected(xn::PoseDetectionCapability& pose, const XnChar* strPose, XnUserID nId, void* pCookie)
{
printf("Pose %s for user %d\n", strPose, nId);
g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
void XN_CALLBACK_TYPE Calibration_Start(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
printf("Starting calibration for user %d\n", nId);
}
void XN_CALLBACK_TYPE Calibration_End(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
if (bSuccess)
{
printf("User calibrated\n");
g_UserGenerator.GetSkeletonCap().StartTracking(nId);
}
else
{
printf("Failed to calibrate user %d\n", nId);
g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(POSE_TO_USE, nId);
}
}
int main()
{  
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
cv::namedWindow("DepthImage",CV_WINDOW_AUTOSIZE);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // 20 joints 
  cloud->width    = 20;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  


XnStatus nRetVal = XN_STATUS_OK;
xn::Context context;
nRetVal = context.Init();
nRetVal = context.OpenFileRecording("Captured.oni");  //Comment for real time kinect data
//nRetVal = depth.Create(context);                    //Uncomment for real time kinect

if(nRetVal)printf("unable to find recording/kinect"); 

nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
CHECK_RC(nRetVal,"No depth");
xn::DepthMetaData g_depthMD ;
g_DepthGenerator.GetMetaData(g_depthMD);
XnMapOutputMode depthMapMode;
g_DepthGenerator.GetMapOutputMode(depthMapMode);
cv::Mat depthmat(depthMapMode.nYRes, depthMapMode.nXRes,CV_16UC1);
// TODO: check error code
// Create the user generator
nRetVal = g_UserGenerator.Create(context);
// TODO: check error code
XnCallbackHandle h1, h2, h3;
g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, h1);
g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(Pose_Detected, NULL, NULL, h2);
g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(Calibration_Start, Calibration_End, NULL, h3);
// Set the profile
g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
// Start generating
nRetVal = context.StartGeneratingAll();
// TODO: check error code



while (TRUE)
{
  cv::Mat rawDepthImage(480,640,CV_16UC1,(void*)g_depthMD.Data());
  convertScaleAbs(rawDepthImage,rawDepthImage, 3.0/256, 0);
  imshow("DepthImage",rawDepthImage);
  
 
nRetVal = context.WaitAndUpdateAll();
if(nRetVal)printf("code error\n");

XnUserID aUsers[15];
XnUInt16 nUsers = 15;
g_UserGenerator.GetUsers(aUsers, nUsers);
for (int i = 0; i < nUsers; ++i)
{
if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
{
XnSkeletonJointPosition Joints[24];
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_HEAD, Joints[0]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_NECK, Joints[1]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_TORSO, Joints[2]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_WAIST, Joints[3]);

g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_COLLAR, Joints[4]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_SHOULDER, Joints[5]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_ELBOW, Joints[6]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_WRIST, Joints[7]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_HAND, Joints[8]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_HIP, Joints[9]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_KNEE, Joints[10]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_FOOT, Joints[11]);

g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_COLLAR, Joints[12]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_SHOULDER, Joints[13]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_ELBOW, Joints[14]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_WRIST, Joints[15]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_HAND, Joints[16]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_HIP, Joints[17]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_KNEE, Joints[18]);
g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_FOOT, Joints[19]);



for (int j = 0; j < cloud->points.size (); ++j)
{
  cloud->points[j].x = Joints[j].position.X/1000;
      cloud->points[j].y = Joints[j].position.Y/1000;
      cloud->points[j].z = Joints[j].position.Z/1000;
}


Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity(); //Transforming skeleton Center of Mass (Torso Joint) to origin
transform_1(0,3)=-Joints[2].position.X/1000;
transform_1(1,3)=-Joints[2].position.Y/1000;
transform_1(2,3)=-Joints[2].position.Z/1000;

pcl::transformPointCloud(*cloud, *cloud, transform_1);


//viewer.showCloud(cloud);
if (!viewer.wasStopped())
         viewer.showCloud (cloud);
     


}

}
if(cv::waitKey(30)==27)
  break;
}

// Clean up
context.Shutdown();
return 0;
}