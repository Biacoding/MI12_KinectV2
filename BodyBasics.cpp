//------------------------------------------------------------------------------
// <copyright file="BodyBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "BodyBasics.h"
#include <iostream>
///////////////////////
#include <dwrite.h>
#pragma comment(lib, "dwrite.lib")
#include <cmath> 

//sensor
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <nlohmann/json.hpp>
#include <sstream>
#include <windows.h>
#include "SafeQueue.h"
#include <mmsystem.h>
#pragma comment(lib, "Winmm.lib")

using boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;
using namespace std::chrono;

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;

float AccSet[6] = { 100 }; // Initialize an array to store all acceleration scalars and information for comparison

INT64 lastTime = 0;
int detect = 3; //Manual delay detection frame rate

const int cache = 60;
bool score[6][cache] = { 0 };
int maxNum = -1;
bool IsactiveRelaxingMusic = true;
bool IsactiveHappyMusic = false;
bool Relax_status = false;
bool Happy_status = false;

int TimesofisHigh = 0;
bool isHigh = false;
int wait_play = 0;


bool keep_music = false;

int DetectNum = -1;
struct VelocityAndAcceleration {
    float Vx_max = 0.0f;
    float Vy_max = 0.0f;
    float Vz_max = 0.0f;
    float Vx_old = 0.0f;
    float Vy_old = 0.0f;
    float Vz_old = 0.0f;
    float Acx_max = 0.0f;
    float Acy_max = 0.0f;
    float Acz_max = 0.0f;
    CameraSpacePoint handRightPosition_old = { 0.0f,0.0f,0.0f };
    float Sum_V = 0.0f;
};
VelocityAndAcceleration VandA[6] = { 0 };  //Store everyone's speed and acceleration information


//It must be before winmain

SafeQueue<std::pair<microseconds, std::string>> dataQueue;//global data queue
float x_value, y_value, z_value; //Store acceleration information in three directions
float Sum_Ac_Sensor = 0.0f; //value from sensor 
void run_server(short port) {
    try {
        boost::asio::io_context ioc;
        tcp::acceptor acceptor(ioc, tcp::endpoint(tcp::v4(), port));
        tcp::socket socket(ioc);

        std::cout << "Server is listening on port " << port << std::endl;

        acceptor.accept(socket);
        std::cout << "Connection accepted from: " << socket.remote_endpoint() << std::endl;

        auto last_time = system_clock::now();

        for (;;) {
            std::vector<char> buffer(1024);
            boost::system::error_code error;

            size_t len = socket.read_some(boost::asio::buffer(buffer), error);

            if (error == boost::asio::error::eof) {
                break; // Connection closed cleanly by peer.
            }
            else if (error) {
                throw boost::system::system_error(error); // Some other error.
            }

            auto now = system_clock::now();
            auto time_diff = duration_cast<microseconds>(now - last_time);
            last_time = now;

            std::string dataReceived(buffer.begin(), buffer.begin() + len);
            dataQueue.push(std::make_pair(time_diff, dataReceived));
        }
    }
    catch (std::exception& e) {
        threadFinished.store(true);
        dataQueue.notify_all();
        std::cerr << "Server error: " << e.what() << std::endl;
    }
    threadFinished.store(true);
}

/// <summary>
/// Get acceleration on mobile phone
/// </summary>
void draw() {
    int i = 0;
    while (!threadFinished.load()) {
        auto dataPair = dataQueue.pop();
        if (!dataPair) {
            break;
        }
        //std::vector<float> raw_values = json["values"].get<std::vector<float>>();
        try {
            auto json = nlohmann::json::parse(dataPair->second);

            x_value = json["x"].get<float>();
            y_value = json["y"].get<float>();
            z_value = json["z"].get<float>();
        }
        catch (const std::exception& e) {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
        }
        Sum_Ac_Sensor = 9.8f * sqrt(x_value * x_value + y_value * y_value + z_value * z_value); //Calculate the acceleration of the mobile phone
        std::wostringstream woss;
        /*woss << "Time difference: " << dataPair->first.count() << " microseconds. Data: " << "x" << x.at(i)
            << ",y" << y.at(i) << ",z" << z.at(i) << std::endl;*/
        woss << "\nSum_Ac_mobile:" << Sum_Ac_Sensor;
        OutputDebugString(woss.str().c_str());
        i++;

        if (wait_play < 60) {
         
            if (Sum_Ac_Sensor > 15.0f) {
                isHigh = true;
            }
            wait_play++;
        }else {
            //play music
            if (IsactiveRelaxingMusic) {
                if (!Relax_status) {
                    PlaySound(TEXT("E:\\sanye.wav"), NULL, SND_FILENAME | SND_ASYNC);
                    Relax_status = true;
                    Happy_status = false;
                }
                IsactiveRelaxingMusic = false;
            }
            else if (IsactiveHappyMusic) {
                if (!Happy_status) {
                    PlaySound(TEXT("E:\\xizao.wav"), NULL, SND_FILENAME | SND_ASYNC);
                    Relax_status = false;
                    Happy_status = true;
                }
                IsactiveHappyMusic = false;
            }
            wait_play = 0;
            if (isHigh) {
                IsactiveHappyMusic = true;
            }
            else {
                IsactiveRelaxingMusic = true;
            }
            isHigh = false;
            
        }
    }
}

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(
    _In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    boost::thread server_thread(run_server, 8080);
    boost::thread drawThread(draw);


    CBodyBasics application;
    application.Run(hInstance, nShowCmd);
    drawThread.join();
}

/// <summary>
/// Constructor
/// </summary>
/// Defines the member variables and methods required to interact with the kinect sensor
CBodyBasics::CBodyBasics() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_pKinectSensor(NULL),
    m_pCoordinateMapper(NULL),
    m_pBodyFrameReader(NULL),
    m_pD2DFactory(NULL),
    m_pRenderTarget(NULL),
    m_pBrushJointTracked(NULL),
    m_pBrushJointInferred(NULL),
    m_pBrushBoneTracked(NULL),
    m_pBrushBoneInferred(NULL),
    m_pBrushHandClosed(NULL),
    m_pBrushHandOpen(NULL),
    m_pBrushHandLasso(NULL),
    m_pDWriteFactory(NULL),//
    m_pTextFormat(NULL)//
{

    InitializeDirectWrite();
    LARGE_INTEGER qpf = { 0 };
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }
}


/// <summary>
/// Destructor
/// </summary>
CBodyBasics::~CBodyBasics()
{
    DiscardDirect2DResources();

    DiscardDirectWriteResources();//************

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with body frame reader
    SafeRelease(m_pBodyFrameReader);

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);

}


HRESULT CBodyBasics::InitializeDirectWrite()
{
    HRESULT hr = DWriteCreateFactory(
        DWRITE_FACTORY_TYPE_SHARED,
        __uuidof(IDWriteFactory),
        reinterpret_cast<IUnknown**>(&m_pDWriteFactory));

    if (SUCCEEDED(hr))
    {
        hr = m_pDWriteFactory->CreateTextFormat(
            L"Segoe UI",                    // Font name
            nullptr,                        // Font collection(nullptr represents the default collection)
            DWRITE_FONT_WEIGHT_NORMAL,         // Font weight
            DWRITE_FONT_STYLE_NORMAL,          // Font style
            DWRITE_FONT_STRETCH_NORMAL,      // font stretching
            20.0f,                             // font size
            L"en-US",                        // locale

            &m_pTextFormat);                         //  Output parameters
    }

    return hr;
}
// Clean up DirectWrite resources
void CBodyBasics::DiscardDirectWriteResources()
{
    SafeRelease(m_pTextFormat);
    SafeRelease(m_pDWriteFactory);
}

/// <summary>
/// Creates the main window and begins processing
/// Run method: This is the main message loop of the application,
/// It handles Windows messagesand draws body posture data in the window.
/// It calls the Update method in the message loop to process the pose data,
/// And use Direct2D to draw the body bonesand hand positions in the application window.
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CBodyBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = { 0 };
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra = DLGWINDOWEXTRA;
    wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc = DefDlgProcW;
    wc.lpszClassName = L"BodyBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CBodyBasics::MessageRouter,
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    // Main message loop
    while (WM_QUIT != msg.message)
    {
     

        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
/// Update: This method is used to update the body posture data.
/// It first checks if the IBodyFrameReader exists,
/// Then try to get the latest body frame. If successful, it processes the body data in the frame,
/// Includes bone position and hand status. 
void CBodyBasics::Update()
{

    if (!m_pBodyFrameReader)
    {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;

        hr = pBodyFrame->get_RelativeTime(&nTime);

        IBody* ppBodies[BODY_COUNT] = { 0 };

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {
            ProcessBody(nTime, BODY_COUNT, ppBodies); //BODY_COUNT=6
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }

    SafeRelease(pBodyFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CBodyBasics* pThis = NULL;

    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CBodyBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CBodyBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);
   
    switch (message)
    {
    case WM_INITDIALOG:
    {
        // Bind application window handle
        m_hWnd = hWnd;

        // Init Direct2D
        D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

        // Get and initialize the default Kinect sensor
        InitializeDefaultSensor();
    }
    break;

    // If the titlebar X is clicked, destroy app
    case WM_CLOSE:
        DestroyWindow(hWnd);
        break;

    case WM_DESTROY:
        // Quit the main message pump
        PostQuitMessage(0);
        break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CBodyBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IBodyFrameSource* pBodyFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
        }

        SafeRelease(pBodyFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param> 帧率
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
/// Process the data for each body frame, updating the on-screen representation
void CBodyBasics::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies)
{
    INT64 currentTime = GetTickCount64();
    float delt_time = (currentTime - lastTime) / 1000.0f; //Convert unit to seconds
    //Check whether a specified time interval has elapsed since last processing (for example, 500 milliseconds)
    if ((currentTime - lastTime) < 50)//50 milliseconds
    {
        return; // If the time interval is too short, skip this frame
    }
    //*******
    if (m_hWnd) 
    {
        HRESULT hr = EnsureDirect2DResources(); 
        if (SUCCEEDED(hr) && m_pRenderTarget && m_pCoordinateMapper)
        {
            m_pRenderTarget->BeginDraw(); //Start drawing
            m_pRenderTarget->Clear();//Clear previous drawing content

            RECT rct;
            GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct); //Gets the size of the video view control's client area, which determines the area for drawing.
            int width = rct.right;
            int height = rct.bottom;

            float MinGap = 10.0f;

            for (int k = 0;k < nBodyCount;k++) {
                if (fabs(AccSet[k] == 0.0f)) {
                    continue;   // Skip people not tested
                }
                if (fabs(AccSet[k] - Sum_Ac_Sensor) < MinGap) { 

                    score[k][detect] = 1;
                    for (int i = 0; i < nBodyCount; i++) {
                        if (i != k) {
                            score[i][detect] = 0;
                        }
                    }

                    MinGap = fabs(AccSet[k] - Sum_Ac_Sensor); // Update minimum gap
                }
            }

            int maxScore = 0;
            
            for (int i = 0; i < nBodyCount; i++) {
                int scoreCount = 0;
                for (int j = 0; j < cache; j++) {
                    scoreCount += score[i][j];
                }

                if (scoreCount > maxScore) {
                    maxScore = scoreCount;
                    maxNum = i;
                }
            }
            DetectNum = maxNum; // Update tagger




            for (int i = 0; i < nBodyCount; ++i)  // Traverse and draw all detected characters
            {
                ID2D1SolidColorBrush* brush = nullptr; // Set color brush
                IBody* pBody = ppBodies[i];
                if (pBody)
                {
                    BOOLEAN bTracked = false;
                    hr = pBody->get_IsTracked(&bTracked);

                    if (SUCCEEDED(hr) && bTracked) 
                    {
                       
                        Joint joints[JointType_Count];
                        D2D1_POINT_2F jointPoints[JointType_Count];
                        HandState leftHandState = HandState_Unknown; 
                        HandState rightHandState = HandState_Unknown;
                       /* JointType_SpineBase(0): The base of the spine, which is the center of the pelvis.
                             JointType_SpineMid(1) : The midpoint of the spine.
                             JointType_Neck(2) : Neck.
                             JointType_Head(3): Head.
                             JointType_ShoulderLeft(4) : Left shoulder.
                             JointType_ElbowLeft(5) : Left elbow.
                             JointType_WristLeft(6) : Left wrist.
                             JointType_HandLeft(7) : Left hand. //Read acceleration
                             JointType_ShoulderRight(8) : Right shoulder.
                             JointType_ElbowRight(9) : Right elbow.
                             JointType_WristRight(10): Right wrist.
                             JointType_HandRight(11) : Right hand. //Read acceleration
                             JointType_HipLeft(12) : Left hip.
                             JointType_KneeLeft(13) : Left knee.
                             JointType_AnkleLeft(14) : Left ankle.
                             JointType_FootLeft(15): Left foot.
                             JointType_HipRight(16) : Right hip.
                             JointType_KneeRight(17) : Right knee.
                             JointType_AnkleRight(18) : Right ankle.
                             JointType_FootRight(19) : Right foot.
                             JointType_SpineShoulder(20): The shoulder of the spine, that is, the upper end of the spine between the shoulders.
                             JointType_HandTipLeft(21): Left finger tip.
                             JointType_ThumbLeft(22) : Left thumb.
                             JointType_HandTipRight(23): Right finger tip.
                             JointType_ThumbRight(24) : Right thumb. */
                             //Need to determine which joint position to obtain
                        pBody->get_HandLeftState(&leftHandState);
                        pBody->get_HandRightState(&rightHandState);
                        hr = pBody->GetJoints(_countof(joints), joints);
                        if (SUCCEEDED(hr))
                        {
                            for (int j = 0; j < _countof(joints); ++j)
                            {
                                jointPoints[j] = BodyToScreen(joints[j].Position, width, height); 
                            }

                            //The following plots the right hand position of each detected person

                            CameraSpacePoint handRightPosition = joints[JointType_HandRight].Position;
                            // Convert to screen coordinates
                            D2D1_POINT_2F screenHandRight = BodyToScreen(handRightPosition, width, height);

                            CameraSpacePoint HeadPosition = joints[JointType_Head].Position;
                           
                            D2D1_POINT_2F screenHead = BodyToScreen(HeadPosition, width, height);


                            // Prepare coordinate text to be displayed
                            WCHAR szHandCoordinates[100]; 
                            

                            
                            float Vx = (handRightPosition.X - VandA[i].handRightPosition_old.X) / delt_time;
                            //if (fabs(Vx) > fabs(VandA[i].Vx_max)) { VandA[i].Vx_max = Vx; }
                            float Vy = (handRightPosition.Y - VandA[i].handRightPosition_old.Y) / delt_time;
                            //if (fabs(Vy) > fabs(VandA[i].Vy_max)) { VandA[i].Vy_max = Vy; }
                            float Vz = (handRightPosition.Z - VandA[i].handRightPosition_old.Z) / delt_time;
                            //if (fabs(Vz) > fabs(VandA[i].Vz_max)) { VandA[i].Vz_max = Vz; }
                            VandA[i].Sum_V = sqrt(Vx * Vx + Vy * Vy + Vz * Vz);

                            float Acx = (Vx - VandA[i].Vx_old) / delt_time;
                            //if (fabs(Acx) > fabs(VandA[i].Acx_max)) { VandA[i].Acx_max = Acx; }
                            float Acy = (Vy - VandA[i].Vy_old) / delt_time;
                            //if (fabs(Acy) > fabs(VandA[i].Acy_max)) { VandA[i].Acy_max = Acy; }
                            float Acz = (Vz - VandA[i].Vz_old) / delt_time;
                            //if (fabs(Acz) > fabs(VandA[i].Acz_max)) { VandA[i].Acz_max = Acz; }
                            float Sum_Axyz = sqrt(Acx * Acx + Acy * Acy + Acz * Acz); 

                            AccSet[i] = Sum_Axyz;

                            //This information should be stored in a global array and compared every frame refresh
                            if (i == DetectNum) { // Identify people
                                //Recognized as holding a mobile phone
                                swprintf_s(szHandCoordinates, L"Num: %d\n Hold the phone\nAc_xyz:%2f", i, Sum_Axyz);
                                brush = m_pBrushPhoneTracked;
                            }
                            else {
                                //swprintf_s(szHandCoordinates, L"Vx: %.2fm/s Vx_max: %.2fm/s Acx: %.2fm/s2 Acx_max: %.2fm/s2 \nVy: %.2fm/s Vy_max: %.2fm/s Acy: %.2fm/s2 Acy_max: %.2fm/s2\nVz: %.2fm/s Vz_max: %.2fm/s Acz: %.2fm/s2 Acz_max: %.2fm/s2\nDelt-Time: %.3fs\n",Vx , VandA[i].Vx_max, Acx, VandA[i].Acx_max, Vy, VandA[i].Vy_max, Acy, VandA[i].Acy_max, Vz, VandA[i].Vz_max, Acz, VandA[i].Acz_max, delt_time);
                                swprintf_s(szHandCoordinates, L"Num: %d\nAcx: %.2fm/s2\nAcy: %.2fm/s2\nAcz: %.2fm/s2\nSum_Ac: %.3fm/s2\n", i, Acx, Acy, Acz, Sum_Axyz);
                                //swprintf_s(szHandCoordinates, L"Num: %d\nVx: %.2fm/s\nVy: %.2fm/s\nVz: %.2fm/s\nSum_V: %.3fm/s\n", i, Vx, Vy, Vz, VandA[i].Sum_V);
                            }
                            // Draw coordinate text
                            if (m_pRenderTarget && m_pTextFormat)
                            {
                                m_pRenderTarget->DrawText(
                                    szHandCoordinates,
                                    ARRAYSIZE(szHandCoordinates),
                                    m_pTextFormat,
                                    //D2D1::RectF(screenHead.x + 10, screenHead.y, screenHead.x + 200, screenHead.y + 40),
                                    D2D1::RectF(screenHead.x - 100, screenHead.y + 500, screenHead.x + 100, screenHead.y - 200),
                                    //D2D1::RectF(10, 0, 650, 40),
                                    m_pBrushJointTracked
                                );
                            }

                            // drawing skeleton
                            DrawBody(joints, jointPoints, brush);
                            DrawHand(leftHandState, jointPoints[JointType_HandLeft]);
                            DrawHand(rightHandState, jointPoints[JointType_HandRight]);

                            lastTime = currentTime; // Update last processing time
                            VandA[i].handRightPosition_old = handRightPosition;
                            VandA[i].Vx_old = Vx;
                            VandA[i].Vy_old = Vy;
                            VandA[i].Vz_old = Vz;
                        }
                    }
                }
            }

            hr = m_pRenderTarget->EndDraw();

            // Device lost, need to recreate the render target
            // We'll dispose it now and retry drawing
            if (D2DERR_RECREATE_TARGET == hr)  // Re-render if device is lost
            {
                hr = S_OK;
                DiscardDirect2DResources();
            }
        }

        if (!m_nStartTime)
        {
            m_nStartTime = nTime; //If it is the first frame, set the start time
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = { 0 };
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow)) 
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f  Time = % I64d", fps, (nTime - m_nStartTime));

        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
        detect++;
        if (detect == cache) detect = 0;
    }
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CBodyBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT CBodyBasics::EnsureDirect2DResources()
{
    HRESULT hr = S_OK;

    if (m_pD2DFactory && !m_pRenderTarget)
    {
        RECT rc;
        GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);

        int width = rc.right - rc.left;
        int height = rc.bottom - rc.top;
        D2D1_SIZE_U size = D2D1::SizeU(width, height);
        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        // Create a Hwnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), size),
            &m_pRenderTarget
        );

        if (FAILED(hr))
        {
            SetStatusMessage(L"Couldn't create Direct2D render target!", 10000, true);
            return hr;
        }

        // light green
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 0.5f), &m_pBrushHandClosed);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 0.5f), &m_pBrushHandOpen);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 0.5f), &m_pBrushHandLasso);
        // Color initialization is done here
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 0.5f), &m_pBrushPhoneTracked);
    }

    return hr;
}

/// <summary>
/// Dispose Direct2d resources 
/// </summary>
void CBodyBasics::DiscardDirect2DResources()
{
    SafeRelease(m_pRenderTarget);

    SafeRelease(m_pBrushJointTracked);
    SafeRelease(m_pBrushJointInferred);
    SafeRelease(m_pBrushBoneTracked);
    SafeRelease(m_pBrushBoneInferred);

    SafeRelease(m_pBrushHandClosed);
    SafeRelease(m_pBrushHandOpen);
    SafeRelease(m_pBrushHandLasso);
}

/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F CBodyBasics::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height) // bodypoint 3D coordinates, length and width of the drawing area
{
    // Calculate the body's position on the screen
    DepthSpacePoint depthPoint = { 0 };
    m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

    float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;  // Proportional scaling screen width width/cDepthWidth
    float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

    return D2D1::Point2F(screenPointX, screenPointY);
}

/// <summary>
/// Draws a body 
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>

void CBodyBasics::DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, ID2D1SolidColorBrush* brush) //Add brush
{
    // Draw the bones
    // Torso
    DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck, brush);
    DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder, brush);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid, brush);
    DrawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase, brush);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight, brush);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft, brush);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight, brush);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft, brush);

    // Right Arm    
    DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight, brush);
    DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight, brush);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight, brush);
    DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight, brush);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight, brush);

    // Left Arm
    DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft, brush);
    DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft, brush);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft, brush);
    DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft, brush);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft, brush);

    // Right Leg
    DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight, brush);
    DrawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight, brush);
    DrawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight, brush);

    // Left Leg
    DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft, brush);
    DrawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft, brush);
    DrawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft, brush);

    // Draw the joints
    for (int i = 0; i < JointType_Count; ++i)
    {
        D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[i], c_JointThickness, c_JointThickness);

        if (pJoints[i].TrackingState == TrackingState_Inferred)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
        }
        else if (pJoints[i].TrackingState == TrackingState_Tracked)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
        }
    }

}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void CBodyBasics::DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1, ID2D1SolidColorBrush* brush)//新增brush来为检测到的人改变颜色
{
    //If no brush is passed in, m_pBrushBoneTracked is used by default.
    if (!brush) {
        brush = m_pBrushBoneTracked;
    }

    //This function needs to add judgment to determine which person outlines different colors. Global variables cannot be set directly. Consider adding an incoming parameter.

    TrackingState joint0State = pJoints[joint0].TrackingState; // Array containing joint data
    TrackingState joint1State = pJoints[joint1].TrackingState;
    //joint0 and joint1 represent the starting point and end point of the joint

    // If we can't find either of these joints, exit
    if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
    {   // Not tracked
        return;
    }

    // Don't draw if both points are inferred
    if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
    {   // speculative tracking
        // If both nodes are in speculative state, give up identification
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
    {   // Tracked
        // If it is recognized that the phone is being held, set the color to m_pBrushPhoneTracked
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], brush, c_TrackedBoneThickness);
        // m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
    }
    else
    {
        // If one is detected and the other is in a speculative state, perform speculative drawing.
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
    }
}

/// <summary>
/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
/// </summary>
/// <param name="handState">state of the hand</param>
/// <param name="handPosition">position of the hand</param>
void CBodyBasics::DrawHand(HandState handState, const D2D1_POINT_2F& handPosition)
{
    D2D1_ELLIPSE ellipse = D2D1::Ellipse(handPosition, c_HandSize, c_HandSize);

    switch (handState)
    {
    case HandState_Closed:
        m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandClosed);
        break;

    case HandState_Open:
        m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandOpen);
        break;

    case HandState_Lasso:
        m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandLasso);
        break;
    }
}





