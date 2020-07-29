/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <unistd.h>
#include <iostream>
#include <fstream>

#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>
#include <fstream>

#include <mutex>

using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase *pKFDB,
                   const string &strSettingPath, const int sensor) : mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0), num_good_constr_predef(200)
{

    //    this->num_good_feature_found = 0;
    //    cout << strSettingPath << endl;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    //    cout << "what" << endl;
    //
    // Load camera parameters from settings file
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    //    cout << "what" << endl;

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);
    //    cout << "what" << endl;

    mbf = fSettings["Camera.bf"];
    camera_fps = fSettings["Camera.fps"];
    if (camera_fps == 0)
        camera_fps = 30;
    cout << "camera fps: " << camera_fps << endl;

    time_track_budget = 1.0 / camera_fps;

    if (sensor != System::STEREO)
    {
        // default code in ORB-SLAM2
        cv::Mat DistCoef(4, 1, CV_32F);
#ifdef USE_FISHEYE_DISTORTION
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.k3"];
        DistCoef.at<float>(3) = fSettings["Camera.k4"];
#else
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
#endif
        DistCoef.copyTo(mDistCoef);

        cout << endl
             << "Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
#ifdef USE_FISHEYE_DISTORTION
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        cout << "- k3: " << DistCoef.at<float>(2) << endl;
        cout << "- k4: " << DistCoef.at<float>(3) << endl;
#else
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if (DistCoef.rows == 5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
#endif

        if (sensor == System::RGBD)
        {
            mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
            cout << endl
                 << "Depth Threshold (Close/Far Points): " << mThDepth << endl;

            mDepthMapFactor = fSettings["DepthMapFactor"];
            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }
    }
    else
    {
        // modified stereo with undistortion of both cameras
        // Load camera parameters from settings file
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fSettings["LEFT.K"] >> K_l;
        if (K_l.type() != CV_32F)
            K_l.convertTo(K_l, CV_32F);
        fSettings["RIGHT.K"] >> K_r;
        if (K_r.type() != CV_32F)
            K_r.convertTo(K_r, CV_32F);

        fSettings["LEFT.P"] >> P_l;
        if (P_l.type() != CV_32F)
            P_l.convertTo(P_l, CV_32F);
        fSettings["RIGHT.P"] >> P_r;
        if (P_r.type() != CV_32F)
            P_r.convertTo(P_r, CV_32F);

        fSettings["LEFT.R"] >> R_l;
        if (R_l.type() != CV_32F)
            R_l.convertTo(R_l, CV_32F);
        fSettings["RIGHT.R"] >> R_r;
        if (R_r.type() != CV_32F)
            R_r.convertTo(R_r, CV_32F);

        fSettings["LEFT.D"] >> D_l;
        if (D_l.type() != CV_32F)
            D_l.convertTo(D_l, CV_32F);
        fSettings["RIGHT.D"] >> D_r;
        if (D_r.type() != CV_32F)
            D_r.convertTo(D_r, CV_32F);

        int rows_l = fSettings["LEFT.height"];
        int cols_l = fSettings["LEFT.width"];
        int rows_r = fSettings["RIGHT.height"];
        int cols_r = fSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return;
        }

        K_l.copyTo(mK_ori);
        D_l.copyTo(mDistCoef);
        R_l.copyTo(mR);
        P_l.copyTo(mP);
        //
        K_r.copyTo(mK_right);
        D_r.copyTo(mDistCoef_right);
        R_r.copyTo(mR_right);
        P_r.copyTo(mP_right);

#ifndef ALTER_STEREO_MATCHING
        //
#ifdef USE_FISHEYE_DISTORTION
        cv::fisheye::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, mMap1_l, mMap2_l);
        cv::fisheye::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, mMap1_r, mMap2_r);
#else
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, mMap1_l, mMap2_l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, mMap1_r, mMap2_r);
#endif

#endif

        cout << endl
             << "Camera Parameters: " << endl;
        cout << "- fx: " << mK.at<float>(0, 0) << endl;
        cout << "- fy: " << mK.at<float>(1, 1) << endl;
        cout << "- cx: " << mK.at<float>(0, 2) << endl;
        cout << "- cy: " << mK.at<float>(1, 2) << endl;
#ifdef USE_FISHEYE_DISTORTION
        cout << "- k1: " << mDistCoef.at<float>(0) << endl;
        cout << "- k2: " << mDistCoef.at<float>(1) << endl;
        cout << "- k3: " << mDistCoef.at<float>(2) << endl;
        cout << "- k4: " << mDistCoef.at<float>(3) << endl;
#else
        cout << "- k1: " << mDistCoef.at<float>(0) << endl;
        cout << "- k2: " << mDistCoef.at<float>(1) << endl;
        if (mDistCoef.rows == 5)
            cout << "- k3: " << mDistCoef.at<float>(4) << endl;
        cout << "- p1: " << mDistCoef.at<float>(2) << endl;
        cout << "- p2: " << mDistCoef.at<float>(3) << endl;
#endif

        mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
        cout << endl
             << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    int nRows = fSettings["Camera2.nRows"];
    int nCols = fSettings["Camera2.nCols"];
    mObsHandler = new Observability(fx, fy, nRows, nCols, cx, cy, 0, 0, sensor);
    if (sensor == System::STEREO)
        mObsHandler->camera.bf = mbf;

#ifdef SPARSE_KEYFRAME_COND
    // Sparse config for key-frames, suited for large-scale (outdoor) environment
    mMinFrames = camera_fps;
    mMaxFrames = camera_fps * 10;
#else
    // Dense (default) config for key-frames, suited for small-scale environment
    mMinFrames = 0;
    mMaxFrames = camera_fps;
    //    mMaxFrames = 18*camera_fps/30;
#endif

#ifdef ENABLE_ANTICIPATION_IN_GRAPH
    //    mVFrameInteval = VIRTUAL_FRAME_STEP / camera_fps;
    mVFrameInteval = VIRTUAL_FRAME_STEP * time_track_budget;
#endif

    mFrameAfterInital = 0;
    mbTrackLossAlert = 0;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;
    if (mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    // default
    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (sensor == System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (sensor == System::MONOCULAR)
    {
        //        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        mpIniORBextractor = new ORBextractor(2000, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }

    cout << endl
         << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

#ifdef DISABLE_RELOC
    std::cout << "Tracking: relocalization disabled!" << std::endl;
#else
    std::cout << "Tracking: relocalization enabled!" << std::endl;
#endif

#ifdef USE_INFO_MATRIX
    mCurrentInfoMat.set_size(7, 7);
#elif defined USE_HYBRID_MATRIX
    mCurrentInfoMat.set_size(13, 13);
#endif
    mCurrentInfoMat.zeros();

    mnInitStereo = 0;

#ifdef ENABLE_DETECTION_IO
    mTimeStamp_loaded = -1;
    mvKeysLeft.clear();
    mvKeysRight.clear();
#endif

    // #ifdef ENABLE_WHITE_BALANCE

    //     // mpBalancer = cv::xphoto::createSimpleWB();
    //     mpBalancer = cv::xphoto::createGrayworldWB();

    // #endif
}

Tracking::~Tracking()
{
    f_realTimeTrack.close();
}

void Tracking::SetRealTimeFileStream(string fNameRealTimeTrack)
{
    f_realTimeTrack.open(fNameRealTimeTrack.c_str());
    f_realTimeTrack << fixed;
    f_realTimeTrack << "#TimeStamp Tx Ty Tz Qx Qy Qz Qw" << std::endl;
}

void Tracking::updateORBExtractor()
{

    assert(mpORBextractorLeft != NULL);

    // take the budget number of input arg as feature extraction constr
    float fScaleFactor = mpORBextractorLeft->GetScaleFactor();
    int nLevels = mpORBextractorLeft->GetLevels();
    int fIniThFAST = mpORBextractorLeft->GetInitThres();
    int fMinThFAST = mpORBextractorLeft->GetMinThres();

    //
    delete mpORBextractorLeft;
    mpORBextractorLeft = new ORBextractor(this->num_good_constr_predef / 2, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (mSensor == System::STEREO)
    {
        assert(mpORBextractorRight != NULL);
        //
        delete mpORBextractorRight;
        mpORBextractorRight = new ORBextractor(this->num_good_constr_predef / 2, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    logCurrentFrame.setZero();

    arma::wall_clock timer_mod;
    timer_mod.tic();

#ifdef ENABLE_WHITE_BALANCE

    // Debug
    cv::equalizeHist(imRectLeft, mImGray);
    cv::Mat imGrayRight;
    cv::equalizeHist(imRectRight, imGrayRight);
    //

    // mpBalancer->balanceWhite(imRectLeft, mImGray);
    // cv::Mat imGrayRight;
    // mpBalancer->balanceWhite(imRectRight, imGrayRight);

#else

    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

#endif

#ifndef ALTER_STEREO_MATCHING
    // undistort the image as in the original ORB pipeline
    cv::remap(mImGray, mImGray, mMap1_l, mMap2_l, cv::INTER_LINEAR);
    cv::remap(imGrayRight, imGrayRight, mMap1_r, mMap2_r, cv::INTER_LINEAR);
#endif

    logCurrentFrame.time_rectification = timer_mod.toc();

    timer_mod.tic();

    if (mImGray.channels() == 3)
    {
        if (mbRGB)
        {
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
        }
    }
    else if (mImGray.channels() == 4)
    {
        if (mbRGB)
        {
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
        }
    }

#ifdef SIMU_MOTION_BLUR

    // Update kernel size for a normalized box filter
    int kernel_size = 5; // 9; // 15;
    cv::Mat kernel = cv::Mat::ones(kernel_size, kernel_size, CV_32F) / (float)(kernel_size * kernel_size);

    cv::filter2D(mImGray, mImGray, -1, kernel);
    cv::filter2D(imGrayRight, imGrayRight, -1, kernel);

#endif

    //    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    //

#ifdef ENABLE_DETECTION_IO
    if (fabs(mTimeStamp_loaded - timestamp) < 1e-6)
    {
        //        cout << "Try creating frame with loaded detection data!" << endl;
        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp,
                              mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK,
                              mK_ori, mDistCoef, mR, mP,
                              mK_right, mDistCoef_right, mR_right, mP_right,
                              mbf, mThDepth, mvKeysLeft, mvKeysRight, mDiscLeft, mDiscRight);
    }
    else
    {
        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp,
                              mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK,
                              mK_ori, mDistCoef, mR, mP,
                              mK_right, mDistCoef_right, mR_right, mP_right,
                              mbf, mThDepth);
    }
#else
    mCurrentFrame = Frame(mImGray, imGrayRight, timestamp,
                          mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK,
                          mK_ori, mDistCoef, mR, mP,
                          mK_right, mDistCoef_right, mR_right, mP_right,
                          mbf, mThDepth);
#endif

    logCurrentFrame.time_ORB_extraction = timer_mod.toc();

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp)
{
    logCurrentFrame.setZero();

    arma::wall_clock timer_mod;
    timer_mod.tic();

#ifdef ENABLE_WHITE_BALANCE

    // Debug
    cv::equalizeHist(imRGB, mImGray);
    cv::Mat imDepth = imD;

#else

    mImGray = imRGB;
    cv::Mat imDepth = imD;

#endif

    if (mImGray.channels() == 3)
    {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
    }
    else if (mImGray.channels() == 4)
    {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
    }

    if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
        imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

    mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

    logCurrentFrame.time_ORB_extraction = timer_mod.toc();

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    logCurrentFrame.setZero();

    arma::wall_clock timer_mod;
    timer_mod.tic();

#ifdef ENABLE_WHITE_BALANCE

    // Debug
    cv::equalizeHist(im, mImGray);

#else

    mImGray = im;

#endif

    if (mImGray.channels() == 3)
    {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
    }
    else if (mImGray.channels() == 4)
    {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
    }

    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
    else {
#ifdef ENABLE_DETECTION_IO
    if (fabs(mTimeStamp_loaded - timestamp) < 1e-6)
    {
        //        cout << "Try creating frame with loaded detection data!" << endl;
        mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, 
            mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mvKeysLeft, mDiscLeft);
    }
    else
    {
        mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, 
            mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
    }
#else
    mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, 
        mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
#endif
    }

    logCurrentFrame.time_ORB_extraction = timer_mod.toc();

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if (mState == NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    arma::wall_clock timer_all, timer_mod;

    mLastProcessedState = mState;
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
    // Update visible map points number
    mNumVisibleMpt = 0;
#endif

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if (mState == NOT_INITIALIZED)
    {
        if (mSensor == System::STEREO || mSensor == System::RGBD)
        {
#if defined ALTER_STEREO_MATCHING && defined DELAYED_STEREO_MATCHING
            mCurrentFrame.PrepareStereoCandidates();
            mCurrentFrame.ComputeStereoMatches_Undistorted(false);
#endif
            StereoInitialization();
        }
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if (mState != OK)
            return;
    }
    else
    {
        timer_all.tic();

        mFrameAfterInital++;

        // set up the time log struct
        logCurrentFrame.frame_time_stamp = mCurrentFrame.mTimeStamp;

        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if (!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if (mState == OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

#if defined DELAYED_STEREO_MATCHING
                mCurrentFrame.PrepareStereoCandidates();
#endif

                if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                {
                    timer_mod.tic();
                    bOK = TrackReferenceKeyFrame();
                                       cout << "number of visible map at TrackReferenceKeyFrame: " << mNumVisibleMpt << endl;
                    if (!bOK)
                        cout << "Track loss at func TrackReferenceKeyFrame !!!" << endl;
                        cout<<"here1<<"<<endl;
                    logCurrentFrame.time_track_frame = timer_mod.toc();
                }
                else
                {
                    timer_mod.tic();
                    bOK = TrackWithMotionModel();
                    //                    cout << "number of visible map at TrackWithMotionModel: " << mNumVisibleMpt << endl;
                    if (!bOK)
                        cout << "Track loss at func TrackWithMotionModel !!!" << endl;
                    logCurrentFrame.time_track_motion = timer_mod.toc();
                    if (!bOK)
                    {
                        timer_mod.tic();
                        bOK = TrackReferenceKeyFrame();
                        if (!bOK)
                            cout << "Track loss at func TrackReferenceKeyFrame !!!" << endl;
                        logCurrentFrame.time_track_frame = timer_mod.toc();
                    }
                }
            }
            else
            {
#ifdef DISABLE_RELOC
                // do nothing
#else

#if defined DELAYED_STEREO_MATCHING
                mCurrentFrame.PrepareStereoCandidates();
#endif

                bOK = Relocalization();
                if (!bOK)
                    cout << "Track loss at func Relocalization !!!" << endl;
#endif
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if (mState == LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if (!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if (!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint *> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if (!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if (bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if (mbVO)
                        {
                            for (int i = 0; i < mCurrentFrame.N; i++)
                            {
                                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if (bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        timer_mod.tic();

        if (!mbOnlyTracking)
        {
            if (bOK)
            {
                bOK = TrackLocalMap();
                //                cout << "number of visible map at TrackLocalMap: " << mNumVisibleMpt << endl;
                if (!bOK)
                    cout << "Track loss at func TrackLocalMap !!!" << endl;
            }
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if (bOK && !mbVO)
            {
                bOK = TrackLocalMap();
            }
        }

        logCurrentFrame.time_track_map = timer_mod.toc();

        if (bOK)
            mState = OK;
        else
            mState = LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if (bOK)
        {
            timer_mod.tic();

            // Update motion model
            if (!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                mVelocity = mCurrentFrame.mTcw * LastTwc;
            }
            else
                mVelocity = cv::Mat();

            // VIZ only
            //        if (mCurrentFrame.mnId % 20 == 0)
            // PlotFrameWithPointMatches();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            logCurrentFrame.time_update_motion = timer_mod.toc();

            //            timer_mod.tic();

            // NOTE
            // When good feature being used, only the selected inliers will be sent into local BA

            //            // Clean VO matches
            //            for(int i=0; i<mCurrentFrame.N; i++)
            //            {
            //                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            //                if(pMP)
            //                    if(pMP->Observations()<1)
            //                    {
            //                        mCurrentFrame.mvbOutlier[i] = false;
            //                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            //                    }
            //            }

            //            // Delete temporal MapPoints
            //            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            //            {
            //                MapPoint* pMP = *lit;
            //                delete pMP;
            //            }
            //            mlpTemporalPoints.clear();

            //            // Check if we need to insert a new keyframe
            //            if(NeedNewKeyFrame()) {
            //                CreateNewKeyFrame();
            //                cout << "New key frame inserted!" << endl;
            //            }

            //            // We allow points with high innovation (considererd outliers by the Huber Function)
            //            // pass to the new keyframe, so that bundle adjustment will finally decide
            //            // if they are outliers or not. We don't want next frame to estimate its position
            //            // with those points so we discard them in the frame.
            //            for(int i=0; i<mCurrentFrame.N;i++)
            //            {
            //                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
            //                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            //            }

            //            logCurrentFrame.time_create_kf = timer_mod.toc();
        }

        // Reset if the camera get lost soon after initialization
        if (mState == LOST)
        {
            if (mpMap->KeyFramesInMap() <= 5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        //        if(!mCurrentFrame.mpReferenceKF)
        //            mCurrentFrame.mpReferenceKF = mpReferenceKF;

#if defined GOOD_FEATURE_MAP_MATCHING
        // predict the pose at next frame
        if (bOK)
        {
            mObsHandler->updatePWLSVec(mLastFrame.mTimeStamp, mLastFrame.mTcw,
                                       mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());
            mObsHandler->predictPWLSVec((mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp), 2);
        }
#endif

        // Since we search for additional matches after publishing the camera state,
        // the creation of last frame should be postponed after the additional matching.
        //        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    //    if(!mCurrentFrame.mTcw.empty())
    //    {
    //        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
    //        mlRelativeFramePoses.push_back(Tcr);
    //        mlpReferences.push_back(mpReferenceKF);
    //        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
    //        mlbLost.push_back(mState==LOST);
    //    }
    //    else
    //    {
    //        // This can happen if tracking is lost
    //        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
    //        mlpReferences.push_back(mlpReferences.back());
    //        mlFrameTimes.push_back(mlFrameTimes.back());
    //        mlbLost.push_back(mState==LOST);
    //    }

    if (!mCurrentFrame.mTcw.empty())
    {
        // Write the real time tracking result to file
        FramePose poseTmp = FramePose(mCurrentFrame.mTimeStamp, mCurrentFrame.mTcw);

        double timestamp = poseTmp.time_stamp;
        cv::Mat Homm = poseTmp.homm;
        cv::Mat R = Homm.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat t = -R * Homm.rowRange(0, 3).col(3);
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);

        f_realTimeTrack << setprecision(6) << timestamp << setprecision(7)
                        << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
                        << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    // unlock the map states?
    //    lock.unlock();
    // std::cout << "what the hell?" << std::endl;

    //
    timer_mod.tic();
    if (mState == OK)
    {
        //        UpdateReferenceKeyFrames();

#ifdef GROUND_TRUTH_GEN_MODE
        double timeCost_sofar = timer_all.toc() + logCurrentFrame.time_ORB_extraction,
                timeCost_rest = time_track_budget * 5 - timeCost_sofar - 0.002;
        // Compute the good feature for local Map
        std::cout << "========== total time to proc 1 frame = " << time_track_budget * 5
                  << "; already taken " << timeCost_sofar
                  << " with ORB time " << logCurrentFrame.time_ORB_extraction
                  << " ; left " << timeCost_rest << " ==========" << std::endl;
#else
        double timeCost_sofar = timer_all.toc() + logCurrentFrame.time_ORB_extraction,
                timeCost_rest = time_track_budget - timeCost_sofar - 0.002;
        // Compute the good feature for local Map
        // std::cout << "========== total time to proc 1 frame = " << time_track_budget
        //           << "; already taken " << timeCost_sofar
        //           << " with ORB time " << logCurrentFrame.time_ORB_extraction
        //           << " ; left " << timeCost_rest << " ==========" << std::endl;
#endif

        if (mObsHandler != NULL)
        {
            // predict info matrix for visible local points
            mObsHandler->mnFrameId = mCurrentFrame.nNextId;

            mObsHandler->mBoundXInFrame = (ORB_SLAM2::Frame::mnMaxX - ORB_SLAM2::Frame::mnMinX) * 0.1; // 20;
            mObsHandler->mBoundYInFrame = (ORB_SLAM2::Frame::mnMaxY - ORB_SLAM2::Frame::mnMinY) * 0.1; // 20;
            mObsHandler->mBoundDepth = 0;
        }

#if defined GOOD_FEATURE_MAP_MATCHING && defined PRECOMPUTE_WITH_MOTION_MODEL
        // predict info matrix for visible local points at next frame
        mObsHandler->mnFrameId = mCurrentFrame.nNextId;
        //  set pred_horizon to 1, so that the NEXT pose is used to compute jacobian
        //        std::thread thread_Predict(&Tracking::PredictJacobianNextFrame, this, timeCost_rest, 1);
#endif

        // Check if we need to insert a new keyframe
#ifdef DELAYED_STEREO_MATCHING
        if (NeedNewKeyFrame_Experimental())
        {
#else
        if (NeedNewKeyFrame())
        {
#endif
            //       if(NeedNewKeyFrame()) {

#if defined DELAYED_MAP_MATCHING
            SearchAdditionalMatchesInFrame(timeCost_rest, mCurrentFrame);
            //            cout << "number of visible map at SearchAdditionalMatchesInFrame: " << mNumVisibleMpt << endl;
#if defined ALTER_STEREO_MATCHING
            timer_mod.tic();
            // perform stereo matching
            int nStereo = mCurrentFrame.ComputeStereoMatches_Undistorted(false);
            //            std::cout << "func Track: number of new points being stereo matched = " << nStereo << "             " << std::endl;
            logCurrentFrame.time_stereo_post = timer_mod.toc();
#endif
#endif
            //            cout << "is Key Frame!" << endl;

            // Clean VO matches
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (pMP)
                    if (pMP->Observations() < 1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }
            }

            CreateNewKeyFrame();
        }
        //        else {
        //            cout << "is Regular Frame!" << endl;
        //        }

#if defined GOOD_FEATURE_MAP_MATCHING && defined PRECOMPUTE_WITH_MOTION_MODEL
        //        thread_Predict.join();
        PredictJacobianNextFrame(MATRIX_BUDGET_PREDICT, 1);
#endif

        // Delete temporal MapPoints
        for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            delete pMP;
        }
        mlpTemporalPoints.clear();

        // We allow points with high innovation (considererd outliers by the Huber Function)
        // pass to the new keyframe, so that bundle adjustment will finally decide
        // if they are outliers or not. We don't want next frame to estimate its position
        // with those points so we discard them in the frame.
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
            if (pMP)
            {
                if (pMP->Observations() < 1)
                {
                    mCurrentFrame.mvbOutlier[i] = false;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
                //
                if (mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }

            //            if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
            //                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
        }

        mLastFrame = Frame(mCurrentFrame);

        if (!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
    logCurrentFrame.time_post_proc = timer_mod.toc();

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if (!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState == LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState == LOST);
    }

    // push the time log of current frame into the vector
    mFrameTimeLog.push_back(logCurrentFrame);
} // namespace ORB_SLAM2

void Tracking::StereoInitialization()
{
    if (mCurrentFrame.N > THRES_INIT_MPT_NUM)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

        // Create KeyFrame
        KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        int count_fails = 0;
        // Create MapPoints and asscoiate to KeyFrame
        cout<<"mCurrentFrame.N: "<<mCurrentFrame.N<<endl;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            cout<<z<<endl;
            if (z > 0)
            {
                //                cout << "Found one valid map point with z = " << z << endl;

                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);
                pNewMP->AddObservation(pKFini, i);
                pKFini->AddMapPoint(pNewMP, i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i] = pNewMP;
            }
            else{
                count_fails++;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;
        cout << "Faild nr of points are: "<<count_fails<<endl; 
        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState = OK;
    }
}

void Tracking::MonocularInitialization()
{

    if (!mpInitializer)
    {
        // Set Reference Frame
        if (mCurrentFrame.mvKeys.size() > THRES_INIT_MPT_NUM)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

            if (mpInitializer)
                delete mpInitializer;

            mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if ((int)mCurrentFrame.mvKeys.size() <= THRES_INIT_MPT_NUM)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9, true);
        // int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);
#ifdef ENABLE_LARGE_SEARCH_WINDOW
        int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 3 * SRH_WINDOW_SIZE_INIT);
#else
        int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, SRH_WINDOW_SIZE_INIT);
#endif

        // Check if there are enough correspondences
        if (nmatches < THRES_INIT_MPT_NUM)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
            return;
        }

        cv::Mat Rcw;                 // Current Camera Rotation
        cv::Mat tcw;                 // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
            {
                if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
                {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
            Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
            tcw.copyTo(Tcw.rowRange(0, 3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
    KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for (size_t i = 0; i < mvIniMatches.size(); i++)
    {
        if (mvIniMatches[i] < 0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

#ifdef GROUND_TRUTH_GEN_MODE
    Optimizer::GlobalBundleAdjustemnt(mpMap, 40);
#else
    Optimizer::GlobalBundleAdjustemnt(mpMap, 20);
#endif

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
    {
        if (vpAllMapPoints[iMP])
        {
            MapPoint *pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for (int i = 0; i < mLastFrame.N; i++)
    {
        MapPoint *pMP = mLastFrame.mvpMapPoints[i];

        if (pMP)
        {
            MapPoint *pRep = pMP->GetReplaced();
            if (pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

bool Tracking::TrackReferenceKeyFrame()
{
    arma::wall_clock timer;

    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector<MapPoint *> vpMapPointMatches;
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
    mNumVisibleMpt = mpReferenceKF->GetMatchNum();
#endif
    int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);
    cout<<"nmatches here are: "<<nmatches<<endl;

    double time_stereo = 0;
#ifdef DELAYED_STEREO_MATCHING
    timer.tic();
    mnInitStereo = mCurrentFrame.ComputeStereoMatches_Undistorted(true);
    cout << "func TrackReferenceKeyFrame: number of points being stereo matched = " << mnInitStereo << "             " << endl;
    time_stereo = timer.toc();
#endif
    //
    logCurrentFrame.time_stereo_frame = time_stereo;

    //    if(nmatches<15)
    if (nmatches < 10)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (mCurrentFrame.mvbOutlier[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    logCurrentFrame.lmk_num_frame = nmatches;

    cout<<"nmatchesMap here are: "<<nmatchesMap<<endl;
    return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame *pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float, int>> vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for (int i = 0; i < mLastFrame.N; i++)
    {
        float z = mLastFrame.mvDepth[i];
        if (z > 0)
        {
            vDepthIdx.push_back(make_pair(z, i));
        }
    }

    if (vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint *pMP = mLastFrame.mvpMapPoints[i];
        if (!pMP)
            bCreateNew = true;
        else if (pMP->Observations() < 1)
        {
            bCreateNew = true;
        }

        if (bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

            mLastFrame.mvpMapPoints[i] = pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > 100)
            break;
    }
}

bool Tracking::PredictRelMotion(const double & time_prev, const double & time_curr,
                                cv::Mat & T_rel) {

    //    cout << "current time: " << time_curr << " vs. " << mvOdomPlanned[0].first << endl;
    //    cout << "previous time: " << time_prev << " vs. " << mvOdomPlanned.back().first << endl;
    if (time_curr < mvOdomPlanned[0].first && time_prev > mvOdomPlanned.back().first)
        return false;

    int Nodom = mvOdomPlanned.size();
    //    cout << "mOdomTrackIdx = " << mOdomTrackIdx << "; Nodom = " << Nodom << endl;
    if (mOdomTrackIdx < 0 || mOdomTrackIdx >= Nodom)
        return false;

    //    cout << mvOdomPlanned[mOdomTrackIdx].first  << " vs. " << time_prev << endl;
    while (mvOdomPlanned[mOdomTrackIdx].first < time_prev) {
        // move forward to the odom wrt time_prev
        mOdomTrackIdx ++;
        if (mOdomTrackIdx == Nodom)
            return false;
    }

    cv::Mat Twc_base;
    cv::Mat Rwc = mvOdomPlanned[mOdomTrackIdx].second.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * mvOdomPlanned[mOdomTrackIdx].second.rowRange(0, 3).col(3);
    Twc_base = cv::Mat::eye(4, 4, CV_32F);
    Rwc.copyTo(Twc_base.rowRange(0, 3).colRange(0, 3));
    twc.copyTo(Twc_base.rowRange(0, 3).col(3));

    //    cout << "Twc_base = " << Twc_base << endl;
    //
    for (int i = mOdomTrackIdx; i < Nodom; ++i) {
        //            cout << setprecision(12) << mvOdomPlanned[i].first << " vs. " << pKF->mTimeStamp + vn * VIRTUAL_KF_INTEVAL << endl;
        // find the closest odom to pKF->mTimeStamp + VIRTUAL_KF_INTEVAL * (vn+1)
        if (mvOdomPlanned[i].first >= time_curr) {
            // jump to the timestamp close to current frame
            mOdomTrackIdx = i;
            //                    Tcw_base = mvOdomPlanned[i].second.inv();
            // Trel = Tcw_current * Twc_prev
            T_rel = mvOdomPlanned[i].second * Twc_base;
            // cv::Mat Ttmp = (Tb2c * T_ed.inv() * T_st * Tc2b);
            return true;
        }
    }

    return false;
}

bool Tracking::TrackWithMotionModel()
{
    arma::wall_clock timer;

    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

#ifdef PRED_WITH_ODOM
    //    cout << "Previous mVelocity = " << endl << mVelocity << endl;
    cv::Mat mVelocity_tmp;
    if (PredictRelMotion(mLastFrame.mTimeStamp, mCurrentFrame.mTimeStamp, mVelocity_tmp)) {
        mVelocity = mVelocity_tmp;
    }
    else {
        // do nothing
    }
    //    cout << "Current mVelocity = " << endl << mVelocity << endl;
#endif

    mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

    std::fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

    // Project points seen in previous frame
    int th;
    if (mSensor != System::STEREO)
        th = 15;
    else
        th = 7;

#ifdef ENABLE_LARGE_SEARCH_WINDOW
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th * 3, mSensor == System::MONOCULAR, mNumVisibleMpt);
#else
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR, mNumVisibleMpt);
#endif
    // int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    double time_stereo = 0;
#ifdef DELAYED_STEREO_MATCHING
    timer.tic();
    mnInitStereo = mCurrentFrame.ComputeStereoMatches_Undistorted(true);
    //    cout << "func TrackWithMotionModel: number of points being stereo matched = " << nStereo << "             "  << endl;
    time_stereo = timer.toc();
#endif
    //
    logCurrentFrame.time_stereo_motion = time_stereo;

    // If few matches, uses a wider window search
    if (nmatches < 20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR, mNumVisibleMpt);
    }

    if (nmatches < 20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (mCurrentFrame.mvbOutlier[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    logCurrentFrame.lmk_num_motion = nmatches;

    if (mbOnlyTracking)
    {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    return nmatchesMap >= 10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    arma::wall_clock timer;

    timer.tic();
    UpdateLocalMap();
    double time_upd_ref = timer.toc();

    timer.tic();
    int mnMatchesInit = SearchLocalPoints();
    double time_srh_ref = timer.toc();

    double time_stereo = 0;
#ifdef DELAYED_STEREO_MATCHING
    timer.tic();
    int nStereo = mCurrentFrame.ComputeStereoMatches_Undistorted(true);
    //    cout << "func TrackLocalMap: number of points being stereo matched = " << nStereo << "             "  << endl;
    time_stereo = timer.toc();
#endif

    timer.tic();
    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;
    double time_opt = timer.toc();

    timer.tic();
    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if (!mbOnlyTracking)
                {
                    if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if (mSensor == System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
        }
    }
    double time_post = timer.toc();

    logCurrentFrame.lmk_num_map = mnMatchesInliers;
    logCurrentFrame.lmk_num_BA = mnMatchesInit / 2;
    logCurrentFrame.time_match = time_upd_ref + time_srh_ref + time_stereo;
    logCurrentFrame.time_optim = time_opt + time_post;
    //
    logCurrentFrame.time_stereo_map = time_stereo;

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    //    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    //        return false;

    //    if(mnMatchesInliers<30)
    //        return false;
    //    else
    //        return true;

    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 25)
        return false;

    if (mnMatchesInliers < 15)
        return false;
    else
        return true;
}

// inject max vol to label the good map points from the rest
void Tracking::PredictJacobianNextFrame(const double time_for_predict, const size_t pred_horizon)
{

    if (time_for_predict <= 0)
    {
        std::cout << "too little budget available!" << std::endl;
        return;
    }
    if (mObsHandler == NULL)
    {
        std::cout << "invalid mObsHandler!" << std::endl;
        return;
    }

    //    arma::wall_clock timer;
    //    timer.tic();
    // find visible map points
    //    std::vector<MapPoint*> tmpMapPoints;
    //    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    //    {
    //        KeyFrame* pKF = *itKF;
    //        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
    //        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
    //        {
    //            MapPoint* pMP = *itMP;
    //            if(!pMP)
    //                continue;
    //            if(pMP->mnUsedForLocalMap==mCurrentFrame.mnId)
    //                continue;
    //            if(!pMP->isBad()) {
    //                //                cv::Mat Pw = pMP->GetWorldPos();
    //                //                if ( mObsHandler->visible_Point_To_Frame(Pw, mObsHandler->kinematic[1].Tcw) == true ) {
    //                //                    tmpMapPoints.push_back(pMP);
    //                //                }
    //                pMP->mnUsedForLocalMap=mCurrentFrame.mnId;
    //                tmpMapPoints.push_back(pMP);
    //            }
    //        }
    //    }
    //    mObsHandler->mMapPoints = &tmpMapPoints;
    //
    //    double time_used = timer.toc();
    //    std::cout << "func RunMapPointsSelection: number of local map points before visibility check = " << tmpMapPoints.size() << std::endl;

    double time_used = 0;
    mObsHandler->mMapPoints = &(mCurrentFrame.mvpMapPoints);

    // select map points with low score
    //    size_t num_map_point_selected = 4000; // 2000; //
    mObsHandler->mKineIdx = pred_horizon;

#ifdef USE_INFO_MATRIX
    mObsHandler->runMatrixBuilding(ORB_SLAM2::MAP_INFO_MATRIX, time_for_predict - time_used, USE_MULTI_THREAD, true);
#elif defined USE_HYBRID_MATRIX
    mObsHandler->runMatrixBuilding(ORB_SLAM2::MAP_HYBRID_MATRIX, time_for_predict - time_used, USE_MULTI_THREAD, true);
#elif defined USE_OBSERVABILITY_MATRIX
    // TODO
#endif

    //    double time_map_bound = timer.toc();
    //    logCurrentFrame.time_mat_pred = time_map_bound;

    //    std::cout << "func PredictJacobianNextFrame: actual time used  = " << time_map_bound << std::endl;
}

void Tracking::BucketingMatches(const Frame *pFrame, vector<GoodPoint> &mpBucketed)
{

    //    int32_t max_features = BUCKET_FEATURE_NUM;
    float bucket_width = BUCKET_WIDTH;
    float bucket_height = BUCKET_HEIGHT;

    // find max values
    float u_max = -99999, v_max = -99999;
    float u_min = 99999, v_min = 99999;
    int32_t inlier_num = 0;
    for (int i = 0; i < pFrame->mvpMapPoints.size(); i++)
    {
        MapPoint *pMP = pFrame->mvpMapPoints[i];
        if (pMP)
        {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true)
            {
                // kpUn.pt.x, kpUn.pt.y;
                cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
                //
                if (kpUn.pt.x > u_max)
                    u_max = kpUn.pt.x;
                if (kpUn.pt.y > v_max)
                    v_max = kpUn.pt.y;
                //
                if (kpUn.pt.x < u_min)
                    u_min = kpUn.pt.x;
                if (kpUn.pt.y < v_min)
                    v_min = kpUn.pt.y;
                //
                inlier_num++;
            }
        }
    }

    //    std::cout << "u_max = " << u_max << "; "  << "v_max = " << v_max << "; " << std::endl;
    //    std::cout << "u_min = " << u_min << "; "  << "v_min = " << v_min << "; " << std::endl;

    // allocate number of buckets needed
    int32_t bucket_cols = (int32_t)floor((u_max - u_min) / float(bucket_width)) + 1;
    int32_t bucket_rows = (int32_t)floor((v_max - v_min) / float(bucket_height)) + 1;
    vector<size_t> *buckets = new vector<size_t>[bucket_cols * bucket_rows];

    //    std::cout << "bucket_cols = " << bucket_cols << "; "  << "bucket_rows = " << bucket_rows << "; " << std::endl;
    //    int32_t max_features = (int32_t)floor( float(inlier_num) * this->ratio_good_inlier_predef / float(bucket_cols*bucket_rows) );
    int32_t max_features = (int32_t)ceil(float(this->num_good_constr_predef / 2) / float(bucket_cols * bucket_rows)) + 1;

    // assign matches to their buckets
    for (int i = 0; i < pFrame->mvpMapPoints.size(); i++)
    {
        MapPoint *pMP = pFrame->mvpMapPoints[i];
        if (pMP)
        {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true)
            {
                //                std::cout << "enter one map point" << std::endl;
                // kpUn.pt.x, kpUn.pt.y;
                cv::KeyPoint kpUn = pFrame->mvKeysUn[i];

                //                std::cout << kpUn.pt.x << "; " << kpUn.pt.y << ";" << bucket_width << "; " << bucket_height << std::endl;

                int32_t u = (int32_t)floor(float(kpUn.pt.x - u_min) / float(bucket_width));
                int32_t v = (int32_t)floor(float(kpUn.pt.y - v_min) / float(bucket_height));

                //                std::cout << "u = " << u << "; v = " << v << std::endl;
                buckets[v * bucket_cols + u].push_back(static_cast<size_t>(i));
            }
        }
    }
    //    std::cout << "fill in content for buckets!" << std::endl;

    // refill p_matched from buckets
    size_t total_num = 0;
    bool stop_bucketing = false;
    mpBucketed.clear();
    for (size_t i = 0; i < bucket_cols * bucket_rows; i++)
    {

        if (stop_bucketing == true)
            break;

        // shuffle bucket indices randomly
        std::random_shuffle(buckets[i].begin(), buckets[i].end());

        // add up to max_features features from this bucket to p_matched
        size_t k = 0;
        for (vector<size_t>::iterator it = buckets[i].begin(); it != buckets[i].end(); it++)
        {
            //
            //            std::cout << "select match " << *it << " from bucket " << i << std::endl;
            GoodPoint tmpLmk(*it, 1);
            mpBucketed.push_back(tmpLmk);
            k++;
            total_num++;
            //
            if (total_num >= this->num_good_constr_predef / 2)
            {
                stop_bucketing = true;
                break;
            }
            if (k >= max_features)
                break;
        }
    }

    //    std::cout << "feature bucketed = " << total_num << std::endl;
    //    std::cout << "done with bucketing!" << std::endl;

    // free buckets
    delete[] buckets;
}

void Tracking::LongLivedMatches(const Frame *pFrame, vector<GoodPoint> &mpLongLived)
{

    mpLongLived.clear();
    for (size_t i = 0; i < pFrame->mvpMapPoints.size(); i++)
    {
        MapPoint *pMP = pFrame->mvpMapPoints[i];
        if (pMP)
        {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true)
            {
                GoodPoint tmpLmk(static_cast<size_t>(i), pMP->mnVisible);
                mpLongLived.push_back(tmpLmk);
            }
        }
    }

    std::sort(mpLongLived.begin(), mpLongLived.end(), GoodPoint::rankObsScore_descend);

    //
    if (mpLongLived.size() > this->num_good_constr_predef / 2)
    {
        mpLongLived.erase(mpLongLived.begin() + this->num_good_constr_predef / 2, mpLongLived.end());
    }
}

bool Tracking::NeedNewKeyFrame_Experimental()
{
    if (mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
        nMinObs = 2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    //    int nNonTrackedClose = 0;
    //    int nTrackedClose= 0;
    //    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
        thRefRatio = 0.4f;

    //    thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = false && (mnMatchesInliers < nRefMatches * 0.25);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio) && mnMatchesInliers > 15);

    if ((c1a || c1b || c1c) && c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if (bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

bool Tracking::NeedNewKeyFrame()
{
    if (mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;
    unsigned long nKFs = mpMap->KeyFramesInMap();

    //    cout << mCurrentFrame.mnId << "; " << mnLastRelocFrameId << "; " << mMaxFrames << "; " << nKFs << endl;
    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
        nMinObs = 2;

    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;
    if (mSensor != System::MONOCULAR)
    {
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)
            {
                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
        thRefRatio = 0.4f;

    if (mSensor == System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

    if ((c1a || c1b || c1c) && c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if (bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR)
            {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if (!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if (mSensor != System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float, int>> vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0)
            {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP)
                    bCreateNew = true;
                else if (pMP->Observations() < 1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }

                if (bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
                    pNewMP->AddObservation(pKF, i);
                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                    break;
            }
        }
    }

#ifdef ENABLE_ANTICIPATION_IN_GRAPH

    // only update visible_mpt_num at this point
    pKF->mNumVisibleMpt = mNumVisibleMpt;

    // Update average matching number
    mpLocalMapper->mParam.avg_match_num = (mpLocalMapper->mParam.avg_match_num +
                                           logCurrentFrame.lmk_num_BA) / 2.0;

    pKF->mvTrel.clear();
    size_t Nodom = mvOdomPlanned.size(), vn = 0;
    //    cout << "Nodom = " << Nodom << endl;
    //    cout << setprecision(12) << "pKF->mTimeStamp + VIRTUAL_KF_INTEVAL = " << pKF->mTimeStamp + VIRTUAL_KF_INTEVAL << endl;
    //    cout << setprecision(12) << "mvOdomPlanned[0].first = " << mvOdomPlanned[0].first << endl;
    //    cout << setprecision(12) << "mvOdomPlanned.back().first = " << mvOdomPlanned.back().first << endl;
    if (pKF->mTimeStamp + mVFrameInteval >= mvOdomPlanned[0].first &&
            pKF->mTimeStamp + mVFrameInteval <= mvOdomPlanned.back().first) {
        // when given valid KF
        cv::Mat Twc_base;
        for (size_t i = mOdomLBAIdx; i < Nodom; ++i) {
            //            cout << setprecision(12) << mvOdomPlanned[i].first << " vs. " << pKF->mTimeStamp + vn * VIRTUAL_KF_INTEVAL << endl;
            // find the closest odom to pKF->mTimeStamp + VIRTUAL_KF_INTEVAL * (vn+1)
            if (mvOdomPlanned[i].first >= pKF->mTimeStamp + vn * mVFrameInteval) {
                if (vn == 0) {
                    mOdomLBAIdx = i;
                    //                    Tcw_base = mvOdomPlanned[i].second.inv();
                    cv::Mat Rwc = mvOdomPlanned[i].second.rowRange(0, 3).colRange(0, 3).t();
                    cv::Mat twc = -Rwc * mvOdomPlanned[i].second.rowRange(0, 3).col(3);
                    Twc_base = cv::Mat::eye(4, 4, CV_32F);
                    Rwc.copyTo(Twc_base.rowRange(0, 3).colRange(0, 3));
                    twc.copyTo(Twc_base.rowRange(0, 3).col(3));
                    //                    Tcw_base.rowRange(0, 3).colRange(0, 3) = mvOdomPlanned[i].second.rowRange(0, 3).colRange(0, 3).t();
                    //                    Tcw_base.rowRange(0, 3).col(3) = -mvOdomPlanned[i].second.rowRange(0, 3).colRange(0, 3) * mvOdomPlanned[i].second.rowRange(0, 3).col(3);

                    vn ++;
                    continue ;
                }

                // Trel = Tcw_current * Twc_prev
                cv::Mat mTrel = mvOdomPlanned[i].second * Twc_base;
                pKF->mvTrel.push_back(mTrel);
                vn ++;
//                std::cout << setprecision(15) << "timeStamp planned: " << mvOdomPlanned[mOdomLBAIdx].first << " to " << mvOdomPlanned[i].first << std::endl;
//                std::cout << setprecision(15) << "timeStamp actual: " << pKF->mTimeStamp << std::endl;
//                std::cout << "mVel planned: " << std::endl
//                          << mTrel << std::endl;
//                std::cout << "mVel actual: " << std::endl
//                          << mVelocity << std::endl;
            }
            if (vn > VIRTUAL_FRAME_NUM)
                break ;
        }
    }

    //    if (mpLocalMapper->mParam.match_ratio_log.size() < NUM_HISTORICAL_BUDGET) {
    //        // no fully loaded yet
    //        mpLocalMapper->mParam.match_ratio_log.push_back(double(logCurrentFrame.lmk_num_BA) / double(mNumVisibleMpt));
    //        mpLocalMapper->mParam.match_ratio_idx ++;
    //    }
    //    else {
    //        // start ring buffering
    //        mpLocalMapper->mParam.match_ratio_log[mpLocalMapper->mParam.match_ratio_idx % NUM_HISTORICAL_BUDGET] =
    //                double(logCurrentFrame.lmk_num_BA) / double(mNumVisibleMpt);
    //        mpLocalMapper->mParam.match_ratio_idx ++;
    //    }
    //    std::cout << "num of map used: " << mNumVisibleMpt
    //              << "; num of matching found: " << logCurrentFrame.lmk_num_BA
    //              << "; matching ratio: " << double(logCurrentFrame.lmk_num_BA) / double(mNumVisibleMpt) << std::endl;
#endif

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchAdditionalMatchesInFrame(const double time_for_search, Frame &F)
{

    if (mObsHandler == NULL || mObsHandler->mLeftMapPoints.size() == 0)
        return;

    arma::wall_clock timer;
    timer.tic();

    if (mObsHandler->mbNeedVizCheck)
    {
        //
        for (vector<MapPoint *>::iterator vit = mObsHandler->mLeftMapPoints.begin(), vend = mObsHandler->mLeftMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP->mnLastFrameSeen == F.mnId)
                continue;
            if (pMP->isBad())
                continue;

            if (timer.toc() > time_for_search / 2.0)
            {
                std::cout << "func SearchAdditionalMatchesInFrame: early stop in visibility check!" << std::endl;
                mObsHandler->mLeftMapPoints.erase(vit, vend);
                break;
            }

            // Project (this fills MapPoint variables for matching)
            if (F.isInFrustum(pMP, 0.5))
            {
                pMP->IncreaseVisible();
                //
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
                mNumVisibleMpt ++;
#endif
            }
        }
    }

    double time_so_far = timer.toc();

    ORBmatcher matcher(0.8);
    double th = 0.5; // 1; // 0.8; // 0.2; //
    if (mbTrackLossAlert == 1)
    {
        std::cout << "func SearchAdditionalMatchesInFrame: increase searching range to avoid track loss !!!" << std::endl;
        th = 1; // 2; // 1.6; // 0.4; //
    }
    int nMatched = matcher.SearchByProjection_Budget(F, mObsHandler->mLeftMapPoints, th, time_for_search - time_so_far);

    //    std::cout << "func SearchAdditionalMatchesInFrame: found " << nMatched
    //              << " additional matches from " << mObsHandler->mLeftMapPoints.size()
    //              << " map points, with total time cost = " << timer.toc() << std::endl;

    logCurrentFrame.lmk_num_BA += nMatched;
}

int Tracking::SearchLocalPoints()
{
    // TODO
    // recall why setting already matching num(which is in constr) to mnInitStereo(which is in pts)???
    //    int nAlreadyMatched = mnInitStereo;
    int nAlreadyMatched = 0;

    arma::wall_clock timer;

#ifdef GOOD_FEATURE_MAP_MATCHING
    //    mCurrentInfoMat.zeros();
    mCurrentInfoMat = arma::eye(size(mCurrentInfoMat)) * 0.00001;
    if (mFrameAfterInital > camera_fps * TIME_INIT_TRACKING && mCurrentFrame.mnId >= mnLastRelocFrameId + 2)
    {
        //    cout << "update pose info in obs class" << endl;
        // NOTE
        // there is no need to do motion prediction again, since it's already be
        // predicted and somewhat optimzed in the 1st stage of pose tracking
        mObsHandler->updatePWLSVec(mLastFrame.mTimeStamp, mLastFrame.mTcw,
                                   mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());

        //    cout << "propagate pose info in obs class" << endl;
        // NOTE
        // instead of using actual time between consequtive frames, we construct virtual frame with slightly longer horizon;
        // the motivation being: reducing the size of matrix to 2-segments (which is the minimim-size); meanwhile preserving the spectral property
        mObsHandler->predictPWLSVec((mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp), 1);

        mObsHandler->mKineIdx = 0;
        mObsHandler->mnFrameId = mCurrentFrame.mnId;

        //#ifdef FRAME_MATCHING_INFO_PRIOR
        //        //    mObsHandler->mMapPoints = &mCurrentFrame.mvpMapPoints;
        //        mObsHandler->pFrame = &mCurrentFrame;
        //        // compute info matrix for frame-by-frame matches
        //#ifdef USE_INFO_MATRIX
        //        mObsHandler->runMatrixBuilding(ORB_SLAM2::FRAME_INFO_MATRIX, MATRIX_BUDGET_REALTIME, USE_MULTI_THREAD, false);
        //#elif defined USE_HYBRID_MATRIX
        //        mObsHandler->runMatrixBuilding(ORB_SLAM2::FRAME_HYBRID_MATRIX, MATRIX_BUDGET_REALTIME, USE_MULTI_THREAD, false);
        //#elif defined USE_OBSERVABILITY_MATRIX
        //        // TODO
        //#endif

        //#endif
    }
#endif
    //    cout << mCurrentInfoMat << endl << endl;

    // Do not search map points already matched
    for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
    {
        MapPoint *pMP = *vit;
        if (pMP)
        {
            if (pMP->isBad())
            {
                *vit = static_cast<MapPoint *>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;

#if defined GOOD_FEATURE_MAP_MATCHING && defined FRAME_MATCHING_INFO_PRIOR
                if (pMP->updateAtFrameId == mCurrentFrame.mnId)
                    mCurrentInfoMat += pMP->ObsMat;
#endif

                //#ifdef ENABLE_ANTICIPATION_IN_BUDGET
                //                mNumVisibleMpt ++;
                //#endif

                nAlreadyMatched += 2;
            }
        }
    }
    //    cout << mCurrentInfoMat << endl << endl;

    int nToMatch = 0;

#if defined GOOD_FEATURE_MAP_MATCHING || defined RANDOM_FEATURE_MAP_MATCHING || defined LONGLIVE_FEATURE_MAP_MATCHING
    //
    mObsHandler->mLeftMapPoints.clear();
    mObsHandler->mbNeedVizCheck = false;
    double time_total_match = 0.015;                                   // 1.0; //
    int num_to_match = this->num_good_constr_predef - nAlreadyMatched; // 50;  //
    if (num_to_match <= 0)
    {
        // skip the rest
        for (size_t i = 0; i < mvpLocalMapPoints.size(); ++i)
        {
            if (mvpLocalMapPoints[i] == NULL)
                continue;
            if (mvpLocalMapPoints[i]->isBad())
                continue;
            if (mvpLocalMapPoints[i]->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (mvpLocalMapPoints[i]->mbTrackInView == false)
                continue;
            //
            mObsHandler->mLeftMapPoints.push_back(mvpLocalMapPoints[i]);
        }
        mObsHandler->mbNeedVizCheck = true;
        //
        return nAlreadyMatched;
    }

    double time_Viz = 0;
    timer.tic();
    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
    {
        MapPoint *pMP = *vit;
        if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if (pMP->isBad())
            continue;

        time_Viz = timer.toc();
        if (time_Viz > time_total_match / 2.0)
        {
            //
            mObsHandler->mLeftMapPoints = vector<MapPoint *>(vit, vend);
            mvpLocalMapPoints.erase(vit, vend);
            mObsHandler->mbNeedVizCheck = true;
            //
            break;
        }

        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(pMP, 0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
            //
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
            mNumVisibleMpt ++;
#endif
        }
    }

    //    cout << "time_Viz = " << time_Viz << "; mvpLocalMapPoints.size() = " << mvpLocalMapPoints.size() << endl;

#else
    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
    {
        MapPoint *pMP = *vit;
        if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if (pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(pMP, 0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
            //
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
            mNumVisibleMpt ++;
#endif
        }
    }

#endif
    //
    // NOTE
    // Here is the place to inject Obs computation; to reduce the load of computation, we might need to approximate exact point Obs with region;
    // Following are the time cost of each step in TrackLocalMap:
    //
    int nMatched = 0;
    if (nToMatch > 0)
    {

#ifdef GOOD_FEATURE_MAP_MATCHING

        timer.tic();

        ORBmatcher matcher(0.8);

        int th = 1; // 1.5; // NOTE try increase the window size for vicon seq
        if (mSensor == System::RGBD)
            th = 3;

        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
        {
            th = 5;
            nMatched = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
        }
        else if (mFrameAfterInital <= camera_fps * TIME_INIT_TRACKING || nToMatch < 400)
        { // 800)
            nMatched = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
            //            cout << "nToMatch = " << nToMatch << "; nMatched = " << nMatched << endl;
        }
        else
        {
            // TEST
            // try computing Jacobian for each map point
            mObsHandler->mMapPoints = &mvpLocalMapPoints;
#ifdef USE_INFO_MATRIX
            mObsHandler->runMatrixBuilding(ORB_SLAM2::MAP_INFO_MATRIX, (time_total_match - time_Viz) / 2, USE_MULTI_THREAD, false);
#elif defined USE_HYBRID_MATRIX
            mObsHandler->runMatrixBuilding(ORB_SLAM2::MAP_HYBRID_MATRIX, (time_total_match - time_Viz) / 2, USE_MULTI_THREAD, false);
#elif defined USE_OBSERVABILITY_MATRIX
            // TODO
#endif

            double time_Mat_Online = timer.toc();
            logCurrentFrame.time_mat_online = time_Mat_Online;
            //            std::cout << "func SearchReferencePointsInFrustum: time cost of matrix building = " << time_Mat_Online << endl;

#ifdef USE_INFO_MATRIX
            nMatched = mObsHandler->runActiveMapMatching(&mCurrentFrame, ORB_SLAM2::FRAME_INFO_MATRIX, mCurrentInfoMat,
                                                         th, matcher, num_to_match, time_total_match - time_Mat_Online - time_Viz);
#elif defined USE_HYBRID_MATRIX
            nMatched = mObsHandler->runActiveMapMatching(&mCurrentFrame, ORB_SLAM2::FRAME_HYBRID_MATRIX, mCurrentInfoMat,
                                                         th, matcher, num_to_match, time_total_match - time_Mat_Online - time_Viz);
#elif defined USE_OBSERVABILITY_MATRIX
            // TODO
#endif
        }
        double time_Match = timer.toc();
        //        std::cout << "func SearchReferencePointsInFrustum: found " << nMatched << " constraints (in total "
        //                  << nMatched + nAlreadyMatched <<  ") in " << time_Match * 1000 << " ms                                 " << endl;
        //        std::cout << "func SearchReferencePointsInFrustum: time cost of active matching = " << time_Match << "; matched feature = " << nMatched << endl;

#elif defined RANDOM_FEATURE_MAP_MATCHING || defined LONGLIVE_FEATURE_MAP_MATCHING

        timer.tic();

        ORBmatcher matcher(0.8);

        int th = 1; // 1.5; // NOTE try increase the window size for vicon seq

        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
        {
            th = 5;
            nMatched = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
        }
        else if (mFrameAfterInital <= camera_fps * TIME_INIT_TRACKING || nToMatch < 400)
        { // 800)
            nMatched = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
            cout << "nToMatch = " << nToMatch << "; nMatched = " << nMatched << endl;
        }
        else
        {
            // TEST
            // try computing Jacobian for each map point
            // print out the time cost
            // double time_total_match = 1.0; // 0.009; //

            mObsHandler->mMapPoints = &mvpLocalMapPoints;

            //
            // int num_to_match = this->num_good_inlier_predef - nMatchesFound; // 50;  //
#ifdef RANDOM_FEATURE_MAP_MATCHING
            nMatched = mObsHandler->runBaselineMapMatching(&mCurrentFrame, ORB_SLAM2::BASELINE_RANDOM,
                                                           th, matcher, num_to_match, time_total_match - time_Viz);
#else
            nMatched = mObsHandler->runBaselineMapMatching(&mCurrentFrame, ORB_SLAM2::BASELINE_LONGLIVE,
                                                           th, matcher, num_to_match, time_total_match - time_Viz);
#endif
        }
        double time_Match = timer.toc();
        std::cout << "func SearchReferencePointsInFrustum: time cost of active matching = " << time_Match << "; matched feature = " << nMatched << endl;

#else

        ORBmatcher matcher(0.8);
        int th = 1;
        if (mSensor == System::RGBD)
            th = 3;
        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
            th = 5;

#ifdef ENABLE_LARGE_SEARCH_WINDOW
        th *= 3;
#endif

        nMatched = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);

#endif
    }

    return nAlreadyMatched + nMatched;
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    //    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();

    logCurrentFrame.lmk_local_map = mvpLocalMapPoints.size();

#ifndef DISABLE_MAP_VIZ
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
#endif
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    unsigned long minFrameId = mCurrentFrame.mnId; // - 2; // - 1; //
    if (mbTrackLossAlert == 2)
        minFrameId = 0;

    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
    {
        KeyFrame *pKF = *itKF;
        const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

        for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
        {
            MapPoint *pMP = *itMP;
            if (!pMP)
                continue;
            if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            if (!pMP->isBad())
            {

#if defined GOOD_FEATURE_MAP_BOUND || defined RANDOM_MAP_BOUND
                //                if (pMP->goodAtFrameId > 0 || mvpLocalMapPoints.size() < 2000) {
                //                    acceptMapPoint = true;
                //                }
                if (pMP->goodAtFrameId >= minFrameId)
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
                else
                {
                    // for those unselected map points, we still keep them and try matching them after current pose being published
                    mvpLocalAdditionalPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
#else
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
#endif
            }
        }
    }
}

void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame *, int> keyframeCounter;
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
            if (!pMP->isBad())
            {
                const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i] = NULL;
            }
        }
    }

    if (keyframeCounter.empty())
    {
        cout << "keyframeCounter is empty!" << endl;
        return;
    }

    int max = 0;
    KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
    {
        KeyFrame *pKF = it->first;

        if (pKF->isBad())
            continue;

        if (it->second > max)
        {
            max = it->second;
            pKFmax = pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if (mvpLocalKeyFrames.size() > 80)
            break;

        KeyFrame *pKF = *itKF;

        const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
        {
            KeyFrame *pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad())
            {
                if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame *> spChilds = pKF->GetChilds();
        for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
        {
            KeyFrame *pChildKF = *sit;
            if (!pChildKF->isBad())
            {
                if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame *pParent = pKF->GetParent();
        if (pParent)
        {
            //            std::cout << pParent << std::endl;
            //            std::cout << pParent->mnTrackReferenceForFrame << std::endl;
            //            std::cout << mCurrentFrame.mnId << std::endl;

            if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                break;
            }
        }
    }

    if (pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    cout << mpKeyFrameDB->mvpKFset.size() << endl;

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if (vpCandidateKFs.empty())
    {
        cout << "failed to find reloc candidate!" << endl;
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<PnPsolver *> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint *>> vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++)
    {
        KeyFrame *pKF = vpCandidateKFs[i];
        if (pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
            if (nmatches < 15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch)
    {
        for (int i = 0; i < nKFs; i++)
        {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver *pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore)
            {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint *> sFound;

                const int np = vbInliers.size();

                for (int j = 0; j < np; j++)
                {
                    if (vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j] = NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if (nGood < 10)
                    continue;

                for (int io = 0; io < mCurrentFrame.N; io++)
                    if (mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if (nGood < 50)
                {
                    int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                    if (nadditional + nGood >= 50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if (nGood > 30 && nGood < 50)
                        {
                            sFound.clear();
                            for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                if (mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                            // Final optimization
                            if (nGood + nadditional >= 50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for (int io = 0; io < mCurrentFrame.N; io++)
                                    if (mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io] = NULL;
                            }
                        }
                    }
                }

                // If the pose is supported by enough inliers stop ransacs and continue
                if (nGood >= 50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch)
    {
        return false;
    }
    else
    {
        int vld_cnt = 0;
        for (int io = 0; io < mCurrentFrame.N; io++)
            if (mCurrentFrame.mvpMapPoints[io])
                vld_cnt++;
        cout << "Reloc succeed with " << vld_cnt << " valid map points!" << endl;

        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }
}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if (mpViewer)
    {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if (mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer *>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if (mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);

#ifdef USE_FISHEYE_DISTORTION
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.k3"];
    DistCoef.at<float>(3) = fSettings["Camera.k4"];
#else
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
#endif

    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

#ifdef ENABLE_DETECTION_IO
//
void Tracking::SaveKeyPoints(const std::string &kpt_path)
{
    cv::FileStorage fs(kpt_path.c_str(), cv::FileStorage::WRITE);
    mCurrentFrame.ExportToYML(fs);
}

// TODO test the function under monocular and rgbd setup;
// make sure it works with empty yaml elements
//void Tracking::LoadKeyPoints(const std::string &kpt_path) {
//    cv::FileStorage fs(kpt_path.c_str(), cv::FileStorage::READ);
//    if(!fs.isOpened()) {
//        cout<<"func LoadKeyPoints: keypoint file not found!"<<endl;
//        mTimeStamp_loaded = -1;
//        mvKeysLeft.clear();
//        mvKeysRight.clear();
//        return ;
//    }

//    //    double val_tmp;
//    //    fs["nNextId"] >> val_tmp; nNextId = (unsigned long)val_tmp;
//    //    fs["mnId"] >> val_tmp; mnId = (unsigned long)val_tmp;

//    fs["mTimeStamp"] >> mTimeStamp_loaded;
//    //    fs["mK"] >> mK;

//    cv::FileNode mvKeysFileNode = fs["mvKeys"];//>> no operator overloading for KeyPoint
//    read(mvKeysFileNode, mvKeysLeft);
//    //    cv::FileNode mvKeysUnFileNode = fs["mvKeysUn"];
//    //    read(mvKeysUnFileNode, mvKeysUn);
//    //    assert(mvKeysUn.size() == mvKeys.size());
//    fs["mDescriptors"] >> mDiscLeft;
//    assert(mvKeysLeft.size() == mDiscLeft.rows);

//    mvKeysFileNode = fs["mvKeysRight"];
//    read(mvKeysFileNode, mvKeysRight);
//    fs["mDescriptorsRight"] >> mDiscRight;
//    assert(mvKeysRight.size() == mDiscRight.rows);

//    //    fs["mvuRight"] >> mvuRight;
//    //    assert(mvuRight.size() == mvKeys.size());
//    //    fs["mvDepth"] >> mvDepth;
//    //    assert(mvDepth.size() == mvKeys.size());
//}

#endif

void Tracking::PlotFrameWithPointMatches()
{

    // create new image to modify it
    cv::Mat img_l_aux;
    mImGray.copyTo(img_l_aux);
    if (img_l_aux.channels() == 1)
        cv::cvtColor(img_l_aux, img_l_aux, CV_GRAY2BGR);

    // Variables
    cv::Point p;
    double thick = 2.0; // 1.0;
    size_t match_counter = 0;
    std::string frame_wid;
    long time_stamp = mCurrentFrame.mTimeStamp * pow(10, 9);

    vector<GoodPoint> mPtObs;

    // Collect the mpSorted
    for (size_t i = 0; i < mCurrentFrame.mvpMapPoints.size(); i++)
    {
        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
        {
            //            if (mCurrentFrame.mvbOutlier[i] == false) {
            GoodPoint tmpLmk(static_cast<size_t>(i), double(mCurrentFrame.mvpMatchScore[i]));
            mPtObs.push_back(tmpLmk);
            //            }
        }
    }

    // Sort features according to the orb matching score
    //    std::sort(mPtObs.begin(), mPtObs.end(), GoodPoint::rankObsScore_ascend);

    // for(size_t i=0; i<mCurrentFrame.mvKeys.size(); i++)  {
    //     cv::KeyPoint kpUn = mCurrentFrame.mvKeys[i];
    //     p = cv::Point( int(kpUn.pt.x),
    //                    int(kpUn.pt.y) );
    //     //
    //     cv::drawMarker( img_l_aux, p, cv::Scalar(255,0,0), cv::MARKER_CROSS, 10, 2);
    // }

    //
    for (size_t i = 0; i < mPtObs.size(); i++)
    {
        cv::KeyPoint kpUn = mCurrentFrame.mvKeys[mPtObs[i].idx];
        p = cv::Point(int(kpUn.pt.x),
                      int(kpUn.pt.y));
        //
        //        if(mCurrentFrame.mvbOutlier[mPtObs[i].idx])
        //        {
        //            cv::circle( img_l_aux, p, 3.0, cv::Scalar(0,0,255), thick);
        //        }
        //        else
        //        {
        //            cv::circle( img_l_aux, p, 3.0, cv::Scalar(0,255,0), thick);
        //        }
        cv::circle(img_l_aux, p, 3.0, cv::Scalar(0, 255, 0), thick);

        match_counter++;

        //
        //        if (match_counter == 50)
        //        {
        //            frame_wid = "/mnt/DATA/tmp/ScreenShot/";
        //            frame_wid += std::to_string(time_stamp);
        //            cv::imwrite( frame_wid + "_m050.png", img_l_aux );
        //        }
        //        else if (match_counter == 100)
        //        {
        //            frame_wid = "/mnt/DATA/tmp/ScreenShot/";
        //            frame_wid += std::to_string(time_stamp);
        //            cv::imwrite( frame_wid + "_m100.png", img_l_aux );
        //        }
        //        else if (match_counter == 150)
        //        {
        //            frame_wid = "/mnt/DATA/tmp/ScreenShot/";
        //            frame_wid += std::to_string(time_stamp);
        //            cv::imwrite( frame_wid + "_m150.png", img_l_aux );
        //        }
    }

    // cv::putText(img_l_aux,
    //             "Keypoints Detected: " + std::to_string(mCurrentFrame.N) + "; Good Features Matched: " + std::to_string(match_counter),
    //             cv::Point(10, 470), // Coordinates
    //             cv::FONT_HERSHEY_PLAIN, // Font
    //             1.5, // Scale. 2.0 = 2x bigger
    //             cv::Scalar(0,0,255), // Color
    //             2.0); // Thickness
    cv::putText(img_l_aux,
                "Features Matched: " + std::to_string(match_counter),
                cv::Point(10, 470),     // Coordinates
                cv::FONT_HERSHEY_PLAIN, // Font
                1.5,                    // Scale. 2.0 = 2x bigger
                cv::Scalar(0, 0, 255),  // Color
                2.0);                   // Thickness

    //
    frame_wid = "/mnt/DATA/tmp/Demo/ORB_blur/";
    frame_wid += std::to_string(time_stamp);
    cv::imwrite(frame_wid + ".png", img_l_aux);

    //    ++ frame_counter;
}

} // namespace ORB_SLAM2
