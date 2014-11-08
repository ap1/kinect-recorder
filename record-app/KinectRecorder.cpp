#include "KinectRecorder.h"

#include <strsafe.h>
#include <vector>

using namespace std;

#define ReleaseIfNotNull(x) {if( x!= NULL) {x->Release(); x = NULL;}}

KinectRecorder::KinectRecorder()
{
  KinectSensor_     = NULL;
  CoordinateMapper_ = NULL;
  BodyFrameReader_  = NULL;
}

KinectRecorder::~KinectRecorder()
{
  ReleaseIfNotNull(BodyFrameReader_);
  ReleaseIfNotNull(CoordinateMapper_);

  if(KinectSensor_) KinectSensor_ -> Close();
  ReleaseIfNotNull(KinectSensor_);
}

void KinectRecorder::Init()
{
  HRESULT hr;

  hr = GetDefaultKinectSensor(&KinectSensor_);

  if (FAILED(hr)) { printf("No ready Kinect found!"); return; }

  if (KinectSensor_)
  {
    IBodyFrameSource* pBodyFrameSource = NULL;

    hr = KinectSensor_->Open();

    if (SUCCEEDED(hr)) { hr = KinectSensor_->get_CoordinateMapper(&CoordinateMapper_); }
    if (SUCCEEDED(hr)) { hr = KinectSensor_->get_BodyFrameSource(&pBodyFrameSource); }
    if (SUCCEEDED(hr)) { hr = pBodyFrameSource->OpenReader(&BodyFrameReader_); }

    ReleaseIfNotNull(pBodyFrameSource);
  }

  if (!KinectSensor_ || FAILED(hr)) { printf("No ready Kinect found!"); return; }

  watch.Reset();
  start_time = 0l;
}

void KinectRecorder::PollForBodyFrame()
{
  if (BodyFrameReader_)
  {
    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = BodyFrameReader_->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr))
    {
      INT64 nTime = 0;

      hr = pBodyFrame->get_RelativeTime(&nTime);

      IBody* ppBodies[BODY_COUNT] = {0};

      if (SUCCEEDED(hr)) { hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies); }
      if (SUCCEEDED(hr)) { ProcessBodyFrame(nTime, ppBodies); }

      for (int i = 0; i < _countof(ppBodies); ++i)
      {
        ReleaseIfNotNull(ppBodies[i]);
      }
    }
    ReleaseIfNotNull(pBodyFrame);
  }
}


void KinectRecorder::ProcessBodyFrame(INT64 nTime, IBody** ppBodies)
{
  HRESULT hr;

  if(start_time == 0l) start_time = nTime;

  int active_body_count = 0;

  for(int bi = 0; bi < BODY_COUNT; bi++)
  {
    if(ppBodies[bi] != NULL)
    {
      BOOLEAN bTracked = false;
      ppBodies[bi]->get_IsTracked(&bTracked);

      if(bTracked) active_body_count++;
    }
  }

  if(active_body_count > 0)
  {
    latest_frame.reset();
    latest_frame.timestamp = nTime;

    //printf("Received %d bodies @ %ld\n", active_body_count, (nTime - start_time));

    for(int bi = 0; bi < BODY_COUNT; bi++)
    {
      if(ppBodies[bi] != NULL)
      {
        BOOLEAN bTracked = false;
        hr = ppBodies[bi]->get_IsTracked(&bTracked);

        if(SUCCEEDED(hr) && bTracked)
        {
          latest_frame.bodies.push_back(BodyInFrame());

          auto& thisbody = latest_frame.bodies.back();

          thisbody.id = bi;
          thisbody.joints.resize(JointType_Count);

          hr = ppBodies[bi]->GetJoints(JointType_Count, joints);

          if(SUCCEEDED(hr))
          {
            for(int ji = 0; ji < JointType_Count; ji++)
            {
              thisbody.joints[ji] = gencvec3f(joints[ji].Position.X, joints[ji].Position.Y, joints[ji].Position.Z);
            }
          } // rec'd joints
        } // body tracked
      } // body ! null
    } // for each body

    if(bRecording)
    {
      recording.frames.push_back(latest_frame);
    }
  } // non zero body count
}