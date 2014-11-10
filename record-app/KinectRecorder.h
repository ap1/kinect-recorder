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

  void    Init();
  void    PollForBodyFrame();
  void    ProcessBodyFrame(INT64 nTime, IBody** ppBodies);
  cvec2f  GetScreenPoint(const cvec3f& pt, int xcenter, int ycenter);
  void    StartRecording();
  void    StopRecording();
  void    SaveToFile(std::string filename);
  cvec3f  Fetch3f(vector<string>& tokens, int loc);
  void    LoadFromFile(std::string filename);

};