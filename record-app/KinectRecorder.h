#pragma once

#include <Kinect.h>

#include "common_code/everything.h"
#include "common_code/FPSMeter.h"

#include <vector>
#include <list>
#include <string>
#include <fstream>

// todo
// - clean up code
// - 

struct BodyInFrame
{
  int id;
  std::vector<cvec3f> joints;
};

struct KinectFrame
{
  INT64 timestamp;
  std::vector<BodyInFrame> bodies;

  inline void reset()
  {
    bodies.clear();
  }
};

struct KinectRecording
{
  std::vector<KinectFrame> frames;

  inline void reset()
  {
    frames.clear();
  }
};

class KinectRecorder
{
public:

  IKinectSensor*      KinectSensor_;
  ICoordinateMapper*  CoordinateMapper_;
  IBodyFrameReader*   BodyFrameReader_;

  Stopwatch watch;

  bool bRecording;

  KinectFrame latest_frame;
  KinectRecording recording;

  INT64 start_time;

  Joint joints[JointType_Count];

  KinectRecorder();
  ~KinectRecorder();

  void Init();
  void PollForBodyFrame();
  void ProcessBodyFrame(INT64 nTime, IBody** ppBodies);

  inline cvec2f GetScreenPoint(const cvec3f& pt, int xcenter, int ycenter)
  {
    DepthSpacePoint depthPoint = { 0 };
    CameraSpacePoint cpt;
    cpt.X = pt.x;
    cpt.Y = pt.y;
    cpt.Z = pt.z;
    CoordinateMapper_->MapCameraPointToDepthSpace(cpt, &depthPoint);
    return gencvec2f(xcenter + (float)depthPoint.X, ycenter + (float)depthPoint.Y);
  }

  inline void start_recording()
  {
    recording.reset();
    bRecording = true;
    printf("starting recording\n");
  }

  inline void stop_recording()
  {
    bRecording = false;
    printf("stop recording\n");
  }

  inline void SaveToFile(std::string filename)
  {
    printf("saving to %s\n", filename.c_str());
    ofstream outfile(filename.c_str());
    if(outfile.good())
    {
      outfile << "TimeStamp,TrackingID";
      outfile << ",SpineBaseX,SpineBaseY,SpineBaseZ";
      outfile << ",SpineMidX,SpineMidY,SpineMidZ";
      outfile << ",NeckX,NeckY,NeckZ";
      outfile << ",HeadX,HeadY,HeadZ";
      outfile << ",ShoulderLeftX,ShoulderLeftY,ShoulderLeftZ";
      outfile << ",ElbowLeftX,ElbowLeftY,ElbowLeftZ";
      outfile << ",WristLeftX,WristLeftY,WristLeftZ";
      outfile << ",HandLeftX,HandLeftY,HandLeftZ";
      outfile << ",ShoulderRightX,ShoulderRightY,ShoulderRightZ";
      outfile << ",ElbowRightX,ElbowRightY,ElbowRightZ";
      outfile << ",WristRightX,WristRightY,WristRightZ";
      outfile << ",HandRightX,HandRightY,HandRightZ";
      outfile << ",HipLeftX,HipLeftY,HipLeftZ";
      outfile << ",KneeLeftX,KneeLeftY,KneeLeftZ";
      outfile << ",AnkleLeftX,AnkleLeftY,AnkleLeftZ";
      outfile << ",FootLeftX,FootLeftY,FootLeftZ";
      outfile << ",HipRightX,HipRightY,HipRightZ";
      outfile << ",KneeRightX,KneeRightY,KneeRightZ";
      outfile << ",AnkleRightX,AnkleRightY,AnkleRightZ";
      outfile << ",FootRightX,FootRightY,FootRightZ";
      outfile << ",SpineShoulderX,SpineShoulderY,SpineShoulderZ";
      outfile << ",HandTipLeftX,HandTipLeftY,HandTipLeftZ";
      outfile << ",ThumbLeftX,ThumbLeftY,ThumbLeftZ";
      outfile << ",HandTipRightX,HandTipRightY,HandTipRightZ";
      outfile << ",ThumbRightX,ThumbRightY,ThumbRightZ";
      outfile << endl;

      for(auto& frame : recording.frames)
      {
        for(auto& body : frame.bodies)
        {
          outfile << frame.timestamp << ",";
          outfile << body.id << ",";
          outfile << body.joints[JointType_SpineBase].x      << "," << body.joints[JointType_SpineBase].y      << "," << body.joints[JointType_SpineBase].z << ",";
          outfile << body.joints[JointType_SpineMid].x       << "," << body.joints[JointType_SpineMid].y       << "," << body.joints[JointType_SpineMid].z << ",";
          outfile << body.joints[JointType_Neck].x           << "," << body.joints[JointType_Neck].y           << "," << body.joints[JointType_Neck].z << ",";
          outfile << body.joints[JointType_Head].x           << "," << body.joints[JointType_Head].y           << "," << body.joints[JointType_Head].z << ",";
          outfile << body.joints[JointType_ShoulderLeft].x   << "," << body.joints[JointType_ShoulderLeft].y   << "," << body.joints[JointType_ShoulderLeft].z << ",";
          outfile << body.joints[JointType_ElbowLeft].x      << "," << body.joints[JointType_ElbowLeft].y      << "," << body.joints[JointType_ElbowLeft].z << ",";
          outfile << body.joints[JointType_WristLeft].x      << "," << body.joints[JointType_WristLeft].y      << "," << body.joints[JointType_WristLeft].z << ",";
          outfile << body.joints[JointType_HandLeft].x       << "," << body.joints[JointType_HandLeft].y       << "," << body.joints[JointType_HandLeft].z << ",";
          outfile << body.joints[JointType_ShoulderRight].x  << "," << body.joints[JointType_ShoulderRight].y  << "," << body.joints[JointType_ShoulderRight].z << ",";
          outfile << body.joints[JointType_ElbowRight].x     << "," << body.joints[JointType_ElbowRight].y     << "," << body.joints[JointType_ElbowRight].z << ",";
          outfile << body.joints[JointType_WristRight].x     << "," << body.joints[JointType_WristRight].y     << "," << body.joints[JointType_WristRight].z << ",";
          outfile << body.joints[JointType_HandRight].x      << "," << body.joints[JointType_HandRight].y      << "," << body.joints[JointType_HandRight].z << ",";
          outfile << body.joints[JointType_HipLeft].x        << "," << body.joints[JointType_HipLeft].y        << "," << body.joints[JointType_HipLeft].z << ",";
          outfile << body.joints[JointType_KneeLeft].x       << "," << body.joints[JointType_KneeLeft].y       << "," << body.joints[JointType_KneeLeft].z << ",";
          outfile << body.joints[JointType_AnkleLeft].x      << "," << body.joints[JointType_AnkleLeft].y      << "," << body.joints[JointType_AnkleLeft].z << ",";
          outfile << body.joints[JointType_FootLeft].x       << "," << body.joints[JointType_FootLeft].y       << "," << body.joints[JointType_FootLeft].z << ",";
          outfile << body.joints[JointType_HipRight].x       << "," << body.joints[JointType_HipRight].y       << "," << body.joints[JointType_HipRight].z << ",";
          outfile << body.joints[JointType_KneeRight].x      << "," << body.joints[JointType_KneeRight].y      << "," << body.joints[JointType_KneeRight].z << ",";
          outfile << body.joints[JointType_AnkleRight].x     << "," << body.joints[JointType_AnkleRight].y     << "," << body.joints[JointType_AnkleRight].z << ",";
          outfile << body.joints[JointType_FootRight].x      << "," << body.joints[JointType_FootRight].y      << "," << body.joints[JointType_FootRight].z << ",";
          outfile << body.joints[JointType_SpineShoulder].x  << "," << body.joints[JointType_SpineShoulder].y  << "," << body.joints[JointType_SpineShoulder].z << ",";
          outfile << body.joints[JointType_HandTipLeft].x    << "," << body.joints[JointType_HandTipLeft].y    << "," << body.joints[JointType_HandTipLeft].z << ",";
          outfile << body.joints[JointType_ThumbLeft].x      << "," << body.joints[JointType_ThumbLeft].y      << "," << body.joints[JointType_ThumbLeft].z << ",";
          outfile << body.joints[JointType_HandTipRight].x   << "," << body.joints[JointType_HandTipRight].y   << "," << body.joints[JointType_HandTipRight].z << ",";
          outfile << body.joints[JointType_ThumbRight].x     << "," << body.joints[JointType_ThumbRight].y     << "," << body.joints[JointType_ThumbRight].z;
          outfile << endl;
        }
      }
      outfile.close();
    }
  }

  inline void LoadFromFile(std::string filename)
  {
    printf("loading from %s\n", filename.c_str());
  }
};