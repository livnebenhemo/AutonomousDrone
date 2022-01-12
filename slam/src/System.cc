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



#include "include/System.h"

namespace ORB_SLAM2 {

    System::System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer) : mSensor(sensor), mpViewer(static_cast<Viewer *>(nullptr)), mbReset(false),
                                            mbActivateLocalizationMode(false),
                                            mbDeactivateLocalizationMode(false) {
        // Output welcome message
        std::cout << std::endl <<
                  "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << std::endl <<
                  "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl <<
                  "This is free software, and you are welcome to redistribute it" << std::endl <<
                  "under certain conditions. See LICENSE.txt." << std::endl << std::endl;

        std::cout << "Input sensor was set to: ";

        if (mSensor == MONOCULAR)
            std::cout << "Monocular" << std::endl;

        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
            exit(-1);
        }


        //Load ORB Vocabulary
        std::cout << std::endl << "Loading ORB Vocabulary. This could take a while..." << std::endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
        if (!bVocLoad) {
            std::cerr << "Wrong path to vocabulary. " << std::endl;
            std::cerr << "Falied to open at: " << strVocFile << std::endl;
            exit(-1);
        }
        std::cout << "Vocabulary loaded!" << std::endl << std::endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create the Map
        mpMap = new Map();

        //Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpMap);
        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                 mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

        //Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
        //mptLocalMapping = new std::thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

        //Initialize the Loop Closing thread and launch
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
        mptLoopClosing = new std::thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

        //Initialize the Viewer thread and launch
        if (bUseViewer) {
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
            mptViewer = new std::thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
        }

        //Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }

    cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp) {
        if (mSensor != MONOCULAR) {
            std::cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << std::endl;
            exit(-1);
        }

        // Check mode change
        {
            std::unique_lock<std::mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            std::unique_lock<std::mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);
        std::unique_lock<std::mutex> lock2(mMutexState);
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;

        return Tcw;
    }

    void System::ActivateLocalizationMode() {
        std::unique_lock<std::mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode() {
        std::unique_lock<std::mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }


    void System::Reset() {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::Shutdown() {
        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();
        if (mpViewer) {
            mpViewer->RequestFinish();
            while (!mpViewer->isFinished())
                usleep(5000);
        }

        // Wait until all thread have effectively stopped
        while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA()) {
            usleep(5000);
        }

        if (mpViewer)
            pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    }

} //namespace ORB_SLAM
