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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <boost/filesystem.hpp>

#include <gflags/gflags.h>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

// General sequences, e.g. EuRoC, TUM-VI, New College, Hololens, Oxford RobotCar
#define TIME_LONG_TO_SEC    1e-9

DEFINE_string(path_to_vocab,    "",     "Dir to ORB vocab file, in txt or bin format");
DEFINE_string(path_to_setting,  "",     "Dir to ORB-SLAM setting file, in yaml format");
DEFINE_string(path_to_sequence, "",     "Dir to stereo image data, containting cam0 and cam1 folders");
DEFINE_string(path_to_planned_track,"", "Dir to planned trajectory, in TUM track format");
DEFINE_string(path_to_track,    "",     "Dir to save pose tracking result");
DEFINE_string(path_to_map,      "",     "Dir to save final mapping result");
DEFINE_uint64(constr_per_frame, 1600,   "Max number of constraints from a mono/stereo frame (keypoint nubmer x 2))");
DEFINE_double(time_start,       0,      "Offset of first frame, in sec");
DEFINE_double(time_duration,    999,    "Duration to last frame, in sec");
DEFINE_double(budget_per_frame, 30,     "Total amount of time to process a frame, in ms");
DEFINE_bool(do_viz,             false,  "Visualize real-time SLAM or not");

// comparison, not case sensitive.
bool compareTimestamp (const std::string& first, const std::string& second)
{
    long time_stamp_l = std::stol( first );
    long time_stamp_r = std::stol( second );
    return ( time_stamp_l < time_stamp_r );
}

bool LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv) {

    google::ParseCommandLineFlags(&argc, &argv, true);

    // Retrieve paths to images
    vector<string> vstrImageLeft_full;
    vector<string> vstrImageRight_full;
    vector<double> vTimeStamp_full;
    LoadImages(FLAGS_path_to_sequence, vstrImageLeft_full, vstrImageRight_full, vTimeStamp_full);

    // Clean images with input duration
    size_t i_st = 0;
    for (; i_st<vTimeStamp_full.size(); ++i_st) {
        if (vTimeStamp_full[i_st] >= vTimeStamp_full[0] + FLAGS_time_start)
            break ;
    }
    size_t i_ed = vTimeStamp_full.size()-1;
    for (; i_ed>=0; --i_ed) {
        if (vTimeStamp_full[i_ed] <= vTimeStamp_full[0] + FLAGS_time_start + FLAGS_time_duration)
            break ;
    }
    if (i_st >= vTimeStamp_full.size() || i_ed <= i_st)
    {
        cerr << "ERROR: No images in provided duration." << endl;
        return 1;
    }
    // Update all images logs
    vector<string> vstrImageLeft(vstrImageLeft_full.begin()+i_st, vstrImageLeft_full.begin()+i_ed+1);
    vector<string> vstrImageRight(vstrImageRight_full.begin()+i_st, vstrImageRight_full.begin()+i_ed+1);
    vector<double> vTimeStamp(vTimeStamp_full.begin()+i_st, vTimeStamp_full.begin()+i_ed+1);
    if(vstrImageLeft.empty() || vstrImageRight.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }
    if(vstrImageLeft.size()!=vstrImageRight.size())
    {
        cerr << "ERROR: Different number of left and right images." << endl;
        return 1;
    }

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(FLAGS_path_to_vocab, FLAGS_path_to_setting, ORB_SLAM2::System::STEREO, FLAGS_do_viz);

    SLAM.SetConstrPerFrame(FLAGS_constr_per_frame);

    // convert budget input from ms to sec
    SLAM.SetBudgetPerFrame(FLAGS_budget_per_frame*1e-3);

#if defined LOGGING_KF_LIST && defined REALTIME_TRAJ_LOGGING

    std::string fNameRealTimeTrack = FLAGS_path_to_track + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    std::string fNameRealTimeBA = FLAGS_path_to_track + "_Log_BA.txt";
    std::cout << std::endl << "Saving BA Log to Log_BA.txt" << std::endl;
    //
    SLAM.SetRealTimeFileStream(fNameRealTimeTrack, fNameRealTimeBA);

#elif defined REALTIME_TRAJ_LOGGING

    std::string fNameRealTimeTrack = FLAGS_path_to_track + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.SetRealTimeFileStream(fNameRealTimeTrack);

#endif

#ifdef ENABLE_MAP_IO
    cout << endl << "-------" << endl;
    cout << "Loading the pre-built map ..." << endl;
    SLAM.LoadMap(FLAGS_path_to_map);
    // SLAM.LoadMap(std::string(argv[8]) + "_Map/");
    SLAM.ForceRelocTracker();
    // SLAM.ForceInitTracker();
#endif


#if defined ENABLE_ANTICIPATION_IN_GRAPH || defined PRED_WITH_ODOM
    cout << endl << "-------" << endl;
    cout << "Loading the planned trajectory ..." << endl;
    SLAM.LoadOdomPlanned(FLAGS_path_to_planned_track);
#endif

    cout << endl << "-------" << endl;
    cout << "Pre-loading all data from the sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    cv::Mat imLeft, imRight;
    vector<cv::Mat> vImLeft, vImRight;
    //
#ifdef ENABLE_DETECTION_IO
    vector<ORB_SLAM2::KeyPointLog> vKptLog;
    cout << "Also loading pre-extracted FAST keypoints from file ..." << endl;
#endif
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    for(int ni=0; ni<nImages; ni++) {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_GRAYSCALE);
        vImLeft.push_back(imLeft);
        imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_GRAYSCALE);
        vImRight.push_back(imRight);

        if(vImLeft[ni].empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(vImRight[ni].empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageRight[ni]) << endl;
            return 1;
        }

#ifdef ENABLE_DETECTION_IO
        // Read feature detection results from file
        std::string strDetectLeft = FLAGS_path_to_sequence + "/cam0/kpt/" + std::to_string(vTimeStamp[ni]) + ".yml";
        // cout << "path to load kpt file: " << strDetectLeft << endl;
        ORB_SLAM2::KeyPointLog kptLog;
        if (!kptLog.ReadFromYAML(strDetectLeft))
        {
            cerr << endl << "Failed to load keypoints at: "
                 << strDetectLeft << endl;
            return 1;
        }
        //
        vKptLog.push_back(kptLog);
#endif
    }

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    //    cout << "Images in the sequence: " << nImages << endl << endl;
    double time_rest = 0;

    // Main loop
    for(int ni=0; ni<nImages; ni++)
    {
        cout << "-------- Frame " << ni << " / " << nImages << " --------" << endl;

        // TODO
        // grab data from pre-load structure
        imLeft = vImLeft[ni];
        imRight = vImRight[ni];
        
#ifdef ENABLE_DETECTION_IO
        SLAM.mpTracker->mTimeStamp_loaded  = vKptLog[ni].mTimeStamp;
        SLAM.mpTracker->mvKeysLeft  = vKptLog[ni].mvKeysLeft;
        SLAM.mpTracker->mvKeysRight = vKptLog[ni].mvKeysRight;
        SLAM.mpTracker->mDiscLeft   = vKptLog[ni].mDiscLeft;
        SLAM.mpTracker->mDiscRight  = vKptLog[ni].mDiscRight;
#endif

        double tframe = vTimeStamp[ni];

        // if (time_rest < -FLAGS_budget_per_frame*1e-3 * 0.5) {
        //     time_rest += FLAGS_budget_per_frame*1e-3;
        //     cout << "Skip frame " << ni << endl;
        //     continue ;
        // }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        // #ifdef ENABLE_DETECTION_IO
        //        // Save feature detection results to file
        //        std::string strDetectLeft = FLAGS_path_to_sequence + "/cam0/kpt/" + std::to_string(vTimeStamp[ni]) + ".yml";
        //        cout << "path to save kpt file: " << strDetectLeft << endl;
        //        SLAM.mpTracker->SaveKeyPoints(strDetectLeft);
        // #endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
        cout<<ttrack<<endl;

        // Wait to load the next frame
        // time_rest = FLAGS_budget_per_frame*1e-3 - ttrack;
        // if(time_rest > 0)
        //     usleep(time_rest*1e6);
    }

    cout << "before calling SLAM.Shutdown !!!" << endl;

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM( FLAGS_path_to_track + "_KeyFrameTrajectory.txt" );
    SLAM.SaveTrackingLog( FLAGS_path_to_track + "_Log.txt" );
#ifdef LOCAL_BA_TIME_LOGGING
    SLAM.SaveMappingLog( FLAGS_path_to_track + "_Log_Mapping.txt" );
#endif
    //
    //    SLAM.SaveLmkLog( FLAGS_path_to_track + "_Log_lmk.txt" );

    /*
#ifdef ENABLE_MAP_IO
 // #ifdef MAP_PUBLISH
 // publish map points
 //  for (size_t i=0; i<10; ++i)
 //   igb.mpMapPub->Refresh();
 // #endif

   SLAM.SaveMap(FLAGS_path_to_map);
   // SLAM.SaveMap(std::string(argv[8]) + "_Map/");

#endif
*/
    
    std::cout << "Finished saving!" << std::endl;

    return 0;
}


bool LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    // setup image directories
    string img_dir_l = strPathToSequence + "/image_0/";
    string img_dir_r = strPathToSequence + "/image_0/";

    // get a sorted list of files in the img directories
    boost::filesystem::path img_dir_path_l(img_dir_l.c_str());
    if (!boost::filesystem::exists(img_dir_path_l))
    {
        cout << endl << "Left image directory does not exist: \t" << img_dir_l << endl;
        return false;
    }
    boost::filesystem::path img_dir_path_r(img_dir_r.c_str());
    if (!boost::filesystem::exists(img_dir_path_r))
    {
        cout << endl << "Right image directory does not exist: \t" << img_dir_r << endl;
        return false;
    }

    // get all files in the img directories
    size_t max_len_l = 0;
    std::list<std::string> imgs_l;
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator file(img_dir_path_l); file != end_itr; ++file)
    {
        boost::filesystem::path filename_path = file->path().filename();
        if (boost::filesystem::is_regular_file(file->status()) &&
                filename_path.extension() == ".png")
        {
            std::string filename(filename_path.string());
            //            cout << filename << endl;
            imgs_l.push_back(filename.substr(0, filename.length()-4));
            //            cout << filename.substr(0, filename.length()-4) << endl;
            max_len_l = max(max_len_l, filename.length());
        }
    }
    size_t max_len_r = 0;
    std::list<std::string> imgs_r;
    for (boost::filesystem::directory_iterator file(img_dir_path_r); file != end_itr; ++file)
    {
        boost::filesystem::path filename_path = file->path().filename();
        if (boost::filesystem::is_regular_file(file->status()) &&
                filename_path.extension() == ".png")
        {
            std::string filename(filename_path.string());
            imgs_r.push_back(filename.substr(0, filename.length()-4));
            max_len_r = max(max_len_r, filename.length());
        }
    }

    imgs_l.sort(compareTimestamp);
    imgs_r.sort(compareTimestamp);

    //    // extract the image files with common name
    //    std::vector<std::string> img_lr_pair; // (n_imgs_l + n_imgs_r);
    //    std::set_intersection (imgs_l.begin(), imgs_l.end(),
    //                           imgs_r.begin(), imgs_r.end(),
    //                           std::back_inserter(img_lr_pair));
    //    std::cout << std::endl << "Number of stereo image pairs: " << img_lr_pair.size() << std::endl;

    std::list<std::string>::iterator it_r = imgs_r.begin();
    for (std::list<std::string>::iterator it_l = imgs_l.begin(); it_l != imgs_l.end(); ++ it_l) {
        while (it_r != imgs_r.end()) {
            // compare the timestamp between i & j
            long time_stamp_l = std::stol( *it_l );
            long time_stamp_r = std::stol( *it_r );
            if (fabs((double)((long double)time_stamp_l * TIME_LONG_TO_SEC) - (double)((long double)time_stamp_r * TIME_LONG_TO_SEC)) < 0.003) {
                // pair left & right frames
                vstrImageLeft.push_back( img_dir_l + *it_l + ".png" );
                vstrImageRight.push_back( img_dir_r + *it_r + ".png" );
                vTimeStamps.push_back( (double)((long double)time_stamp_l * TIME_LONG_TO_SEC) );
                //                cout << vTimeStamps[vTimeStamps.size() - 1] << endl;
                // move on to the next left index
                it_r ++;
                break ;
            }
            else if ((double)((long double)time_stamp_l * TIME_LONG_TO_SEC) < (double)((long double)time_stamp_r * TIME_LONG_TO_SEC)) {
                // skip current left index
                break ;
            }
            else {
                // move on to the next right index
                it_r ++;
            }
        }
    }
    assert(vstrImageLeft.size() == vstrImageRight.size());
    std::cout << std::endl << "Number of stereo image pairs: " << vstrImageRight.size() << std::endl;

    return true;
}
