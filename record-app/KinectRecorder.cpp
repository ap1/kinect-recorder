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

cvec2f KinectRecorder::GetScreenPoint(const cvec3f& pt, int xcenter, int ycenter)
{
  DepthSpacePoint depthPoint = { 0 };
  CameraSpacePoint cpt;
  cpt.X = pt.x;
  cpt.Y = pt.y;
  cpt.Z = pt.z;
  CoordinateMapper_->MapCameraPointToDepthSpace(cpt, &depthPoint);
  return gencvec2f(xcenter + (float)depthPoint.X, ycenter + (float)depthPoint.Y);
}

void KinectRecorder::StartRecording()
{
  recording.reset();
  bRecording = true;
  printf("starting recording\n");
}

void KinectRecorder::StopRecording()
{
  bRecording = false;
  printf("stop recording\n");
}

void KinectRecorder::SaveToFile(std::string filename)
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

cvec3f KinectRecorder::Fetch3f(vector<string>& tokens, int loc)
{
  cvec3f ret = gencvec3f(
    lexical_cast<float>(tokens[loc + 0]),
    lexical_cast<float>(tokens[loc + 1]),
    lexical_cast<float>(tokens[loc + 2]));
  return ret;
}

void KinectRecorder::LoadFromFile(std::string filename)
{
  printf("loading from %s\n", filename.c_str());
  ifstream infile(filename.c_str());
  if(infile.good())
  {
    string line;

    getline(infile, line); // skip first line

    INT64 last_timestamp = 0;

    recording.reset(); // clear any existing frames

    while(!infile.eof())
    {
      vector<string> tokens;
      getline(infile, line);
      vtokenize_degen(line, ",", tokens);
      if(tokens.size() == (2 + JointType_Count * 3))
      {
        INT64 timestamp = lexical_cast<INT64>(tokens[0]);
        int bodyid = lexical_cast<int>(tokens[1]);

        //printf("reading frame %ld, %d\n", timestamp, bodyid);

        if(timestamp != last_timestamp)
        {
          recording.frames.push_back(KinectFrame());
          recording.frames.back().timestamp = timestamp;
        }

        auto& curframe = recording.frames.back();

        curframe.bodies.push_back(BodyInFrame());

        auto& body = curframe.bodies.back();

        body.id = bodyid;
        body.joints.resize(JointType_Count);

        body.joints[JointType_SpineBase]      = Fetch3f(tokens, 2 + 3 * JointType_SpineBase);
        body.joints[JointType_SpineMid]       = Fetch3f(tokens, 2 + 3 * JointType_SpineMid);
        body.joints[JointType_Neck]           = Fetch3f(tokens, 2 + 3 * JointType_Neck);
        body.joints[JointType_Head]           = Fetch3f(tokens, 2 + 3 * JointType_Head);
        body.joints[JointType_ShoulderLeft]   = Fetch3f(tokens, 2 + 3 * JointType_ShoulderLeft);
        body.joints[JointType_ElbowLeft]      = Fetch3f(tokens, 2 + 3 * JointType_ElbowLeft);
        body.joints[JointType_WristLeft]      = Fetch3f(tokens, 2 + 3 * JointType_WristLeft);
        body.joints[JointType_HandLeft]       = Fetch3f(tokens, 2 + 3 * JointType_HandLeft);
        body.joints[JointType_ShoulderRight]  = Fetch3f(tokens, 2 + 3 * JointType_ShoulderRight);
        body.joints[JointType_ElbowRight]     = Fetch3f(tokens, 2 + 3 * JointType_ElbowRight);
        body.joints[JointType_WristRight]     = Fetch3f(tokens, 2 + 3 * JointType_WristRight);
        body.joints[JointType_HandRight]      = Fetch3f(tokens, 2 + 3 * JointType_HandRight);
        body.joints[JointType_HipLeft]        = Fetch3f(tokens, 2 + 3 * JointType_HipLeft);
        body.joints[JointType_KneeLeft]       = Fetch3f(tokens, 2 + 3 * JointType_KneeLeft);
        body.joints[JointType_AnkleLeft]      = Fetch3f(tokens, 2 + 3 * JointType_AnkleLeft);
        body.joints[JointType_FootLeft]       = Fetch3f(tokens, 2 + 3 * JointType_FootLeft);
        body.joints[JointType_HipRight]       = Fetch3f(tokens, 2 + 3 * JointType_HipRight);
        body.joints[JointType_KneeRight]      = Fetch3f(tokens, 2 + 3 * JointType_KneeRight);
        body.joints[JointType_AnkleRight]     = Fetch3f(tokens, 2 + 3 * JointType_AnkleRight);
        body.joints[JointType_FootRight]      = Fetch3f(tokens, 2 + 3 * JointType_FootRight);
        body.joints[JointType_SpineShoulder]  = Fetch3f(tokens, 2 + 3 * JointType_SpineShoulder);
        body.joints[JointType_HandTipLeft]    = Fetch3f(tokens, 2 + 3 * JointType_HandTipLeft);
        body.joints[JointType_ThumbLeft]      = Fetch3f(tokens, 2 + 3 * JointType_ThumbLeft);
        body.joints[JointType_HandTipRight]   = Fetch3f(tokens, 2 + 3 * JointType_HandTipRight);
        body.joints[JointType_ThumbRight]     = Fetch3f(tokens, 2 + 3 * JointType_ThumbRight);

        last_timestamp = timestamp;
      }
      else
      {
        if(tokens.size() > 0)
        {
          printf("We require %d columns (received %d)!\n", (2 + JointType_Count * 3), tokens.size());
        }
      }
    }

    infile.close();

    printf("Loaded %d frames\n", recording.frames.size());
  }
}