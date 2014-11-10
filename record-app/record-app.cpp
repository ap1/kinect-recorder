// record-app.cpp : Defines the entry point for the application.
//

#define WIN32_LEAN_AND_MEAN

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>
#include <commdlg.h>

#include <string>

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

// TODO: reference additional headers your program requires here

#include "common_code\everything.h"
#include "common_code\FPSMeter.h"

#include "record-app.h"
#include "KinectRecorder.h"


eAppState state;
KinectRecorder recorder;
Stopwatch watch;


void DrawRect(HDC& hdc, cvec2f& beg, cvec2f& end)
{
  MoveToEx(hdc, (int)beg.x, (int)beg.y, NULL);
  LineTo  (hdc, (int)end.x, (int)beg.y);
  LineTo  (hdc, (int)end.x, (int)end.y);
  LineTo  (hdc, (int)beg.x, (int)end.y);
  LineTo  (hdc, (int)beg.x, (int)beg.y);
}

void DrawLine(HDC& hdc, cvec2f& beg, cvec2f& end)
{
  MoveToEx(hdc, (int)beg.x, (int)beg.y, NULL);
  LineTo  (hdc, (int)end.x, (int)end.y);
}

void DrawBone(HDC& hdc, const BodyInFrame& b, JointType j0, JointType j1)
{
  cvec3f pos0 = b.joints[j0];
  cvec3f pos1 = b.joints[j1];

  DrawLine(hdc, 
    recorder.GetScreenPoint(pos0, 0, 0), 
    recorder.GetScreenPoint(pos1, 0, 0));
}

void DrawBody(HDC& hdc, const BodyInFrame& b)
{
  // Draw the bones

  // Torso
  DrawBone(hdc, b, JointType_Head,          JointType_Neck);
  DrawBone(hdc, b, JointType_Neck,          JointType_SpineShoulder);
  DrawBone(hdc, b, JointType_SpineShoulder, JointType_SpineMid);
  DrawBone(hdc, b, JointType_SpineMid,      JointType_SpineBase);
  DrawBone(hdc, b, JointType_SpineShoulder, JointType_ShoulderRight);
  DrawBone(hdc, b, JointType_SpineShoulder, JointType_ShoulderLeft);
  DrawBone(hdc, b, JointType_SpineBase,     JointType_HipRight);
  DrawBone(hdc, b, JointType_SpineBase,     JointType_HipLeft);
  // Right Arm    
  DrawBone(hdc, b, JointType_ShoulderRight, JointType_ElbowRight);
  DrawBone(hdc, b, JointType_ElbowRight,    JointType_WristRight);
  DrawBone(hdc, b, JointType_WristRight,    JointType_HandRight);
  DrawBone(hdc, b, JointType_HandRight,     JointType_HandTipRight);
  DrawBone(hdc, b, JointType_WristRight,    JointType_ThumbRight);
  // Left Arm
  DrawBone(hdc, b, JointType_ShoulderLeft,  JointType_ElbowLeft);
  DrawBone(hdc, b, JointType_ElbowLeft,     JointType_WristLeft);
  DrawBone(hdc, b, JointType_WristLeft,     JointType_HandLeft);
  DrawBone(hdc, b, JointType_HandLeft,      JointType_HandTipLeft);
  DrawBone(hdc, b, JointType_WristLeft,     JointType_ThumbLeft);
  // Right Leg
  DrawBone(hdc, b, JointType_HipRight,      JointType_KneeRight);
  DrawBone(hdc, b, JointType_KneeRight,     JointType_AnkleRight);
  DrawBone(hdc, b, JointType_AnkleRight,    JointType_FootRight);
  // Left Leg
  DrawBone(hdc, b, JointType_HipLeft,       JointType_KneeLeft);
  DrawBone(hdc, b, JointType_KneeLeft,      JointType_AnkleLeft);
  DrawBone(hdc, b, JointType_AnkleLeft,     JointType_FootLeft);
}

#define MAX_LOADSTRING 512

// Global Variables:
HINSTANCE hInst;								// current instance
HWND mainhWnd;
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY MyWinMain(_In_ HINSTANCE hInstance,
  _In_opt_ HINSTANCE hPrevInstance,
  _In_ LPTSTR    lpCmdLine,
  _In_ int       nCmdShow)
{
  UNREFERENCED_PARAMETER(hPrevInstance);
  UNREFERENCED_PARAMETER(lpCmdLine);

  // TODO: Place code here.
  MSG msg;
  HACCEL hAccelTable;

  // Initialize global strings
  LoadString(hInstance, IDC_RECORDAPP, szWindowClass, MAX_LOADSTRING);
  MyRegisterClass(hInstance);

  state = eAppState::Idle;
  watch.Reset();
  recorder.Init();

  // Perform application initialization:
  if (!InitInstance(hInstance, nCmdShow))
  {
    return FALSE;
  }

  hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_RECORDAPP));

  BOOL bRet;

  while (state != eAppState::Done)
  {
    if (watch.GetTime() > 0.01f)
    {
      InvalidateRect(mainhWnd, NULL, true);
      watch.Reset();
    }

    recorder.PollForBodyFrame();

    if ((bRet = PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) != 0)
    {
      if (bRet == -1)
      {
      }
      else if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
      {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
      }
    }
  }

  return (int)msg.wParam;
}



//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
  WNDCLASSEX wcex;

  wcex.cbSize = sizeof(WNDCLASSEX);

  wcex.style = CS_HREDRAW | CS_VREDRAW;
  wcex.lpfnWndProc = WndProc;
  wcex.cbClsExtra = 0;
  wcex.cbWndExtra = 0;
  wcex.hInstance = hInstance;
  wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_RECORDAPP));
  wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
  wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
  wcex.lpszMenuName = MAKEINTRESOURCE(IDC_RECORDAPP);
  wcex.lpszClassName = szWindowClass;
  wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

  return RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
  hInst = hInstance; // Store instance handle in our global variable

  mainhWnd = CreateWindow(szWindowClass, L"Kinect Recorder", WS_OVERLAPPEDWINDOW,
    CW_USEDEFAULT, CW_USEDEFAULT, 800, 600, NULL, NULL, hInstance, NULL);

  if (!mainhWnd)
  {
    return FALSE;
  }

  ShowWindow(mainhWnd, nCmdShow);
  UpdateWindow(mainhWnd);

  return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND	- process the application menu
//  WM_PAINT	- Paint the main window
//  WM_DESTROY	- post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  int wmId, wmEvent;
  PAINTSTRUCT ps;
  HDC hdc;
  
  switch (message)
  {
  case WM_COMMAND:
    wmId = LOWORD(wParam);
    wmEvent = HIWORD(wParam);
    // Parse the menu selections:
    switch (wmId)
    {
      case IDM_NEW:
      break;
      case IDM_OPEN:
      {
        OPENFILENAME ofn;

        char openFileName[MAX_PATH] = "";

        ZeroMemory(&ofn, sizeof(ofn));

        ofn.lStructSize = sizeof(ofn); 
        ofn.hwndOwner = mainhWnd;
        ofn.lpstrFilter = (LPCWSTR)L"Comma-Separated Values (*.csv)\0*.csv\0All Files (*.*)\0*.*\0";
        ofn.lpstrFile = (LPWSTR)openFileName;
        ofn.nMaxFile = MAX_PATH;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;
        ofn.lpstrDefExt = (LPCWSTR)L"csv";

        GetOpenFileName(&ofn);
        wcstombs (openFileName, ofn.lpstrFile, MAX_PATH);

        recorder.LoadFromFile(openFileName);
      }
      break;
      case IDM_SAVE:
      {
        OPENFILENAME ofn;

        char saveFileName[MAX_PATH] = "";

        ZeroMemory(&ofn, sizeof(ofn));

        ofn.lStructSize = sizeof(ofn); 
        ofn.hwndOwner = mainhWnd;
        ofn.lpstrFilter = (LPCWSTR)L"Comma-Separated Values (*.csv)\0*.csv\0All Files (*.*)\0*.*\0";
        ofn.lpstrFile = (LPWSTR)saveFileName;
        ofn.nMaxFile = MAX_PATH;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;
        ofn.lpstrDefExt = (LPCWSTR)L"csv";

        GetSaveFileName(&ofn);
        wcstombs (saveFileName, ofn.lpstrFile, MAX_PATH);

        recorder.SaveToFile(saveFileName);
      }
      break;
      case IDM_REC_START: recorder.StartRecording();
      break;
      case IDM_REC_STOP:  recorder.StopRecording();
      break;
      case IDM_PLAY:
        state = (state == eAppState::Playing) ? eAppState::Idle : eAppState::Playing;
      break;
    case IDM_ABOUT:
      DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
      break;
    case IDM_EXIT:
      state = eAppState::Done;
      DestroyWindow(hWnd);
      break;
    default:
      return DefWindowProc(hWnd, message, wParam, lParam);
    }
    break;
  case WM_PAINT:
    hdc = BeginPaint(hWnd, &ps);
    if(state == eAppState::Playing)
    {
      static int frameid = 0;
      if (frameid < (int)recorder.recording.frames.size())
      {
        auto& curframe = recorder.recording.frames[frameid];
        if(curframe.bodies.size() > 0)
        {
          DrawBody(hdc, curframe.bodies[0]);
        }
      }
      frameid = (frameid + 1) % (int)recorder.recording.frames.size();
    }
    else
    {
      if(recorder.latest_frame.bodies.size() > 0)
      {
        DrawBody(hdc, recorder.latest_frame.bodies[0]);
      }
    }
    EndPaint(hWnd, &ps);
    break;
  case WM_DESTROY:
    PostQuitMessage(0);
    break;
  default:
    return DefWindowProc(hWnd, message, wParam, lParam);
  }
  return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
  UNREFERENCED_PARAMETER(lParam);
  switch (message)
  {
  case WM_INITDIALOG:
    return (INT_PTR)TRUE;

  case WM_COMMAND:
    if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
    {
      EndDialog(hDlg, LOWORD(wParam));
      return (INT_PTR)TRUE;
    }
    break;
  }
  return (INT_PTR)FALSE;
}



int main()
{
  return MyWinMain(GetModuleHandle(NULL), NULL, GetCommandLine(), SW_SHOW);
}