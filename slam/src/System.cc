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



#include "System.h"

namespace ORB_SLAM2 {

    System::System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer, bool bReuse, std::string mapName, bool continue_mapping,
                   bool isPangolinExists) : mSensor(sensor), mbReset(false), mbActivateLocalizationMode(false),
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

        mpVocabulary = std::make_shared<ORBVocabulary>();
        bool bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
        if (!bVocLoad) {
            std::cerr << "Wrong path to vocabulary. " << std::endl;
            std::cerr << "Falied to open at: " << strVocFile << std::endl;
            exit(-1);
        }
        std::cout << "Vocabulary loaded!" << std::endl << std::endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = std::make_shared<KeyFrameDatabase>(*mpVocabulary);//new KeyFrameDatabase(mpVocabulary);

        //Create the Map
        if (!bReuse) {
            mpMap = std::make_shared<Map>();
        }
        if (bReuse) {
            LoadMap(mapName);
            //mpKeyFrameDatabase->set_vocab(mpVocabulary);
            auto vpKFs = mpMap->GetAllKeyFrames();
            for (auto it = vpKFs.begin(); it != vpKFs.end(); ++it) {
                (*it)->SetKeyFrameDatabase(mpKeyFrameDatabase);
                (*it)->SetORBvocabulary(mpVocabulary);
                (*it)->SetMap(mpMap);
                (*it)->ComputeBoW();
                mpKeyFrameDatabase->add(*it);
                auto mapPoints = mpMap->GetAllMapPoints();
                (*it)->SetMapPoints(mapPoints);
                (*it)->SetSpanningTree(vpKFs);
                (*it)->SetGridParams(vpKFs);
                // Reconstruct map points Observation
            }

            auto vpMPs = mpMap->GetAllMapPoints();
            for (auto mit = vpMPs.begin(); mit != vpMPs.end(); ++mit) {
                (*mit)->SetMap(mpMap);
                (*mit)->SetObservations(vpKFs);

            }

            for (auto it = vpKFs.begin(); it != vpKFs.end(); ++it) {
                (*it)->UpdateConnections();
            }


        }
        std::cout << std::endl << mpMap << " : is the created map address" << std::endl;
        //Create Drawers. These are used by the Viewer
        mpFrameDrawer = std::make_shared<FrameDrawer>(mpMap.get());
        mpMapDrawer = std::make_shared<MapDrawer>(mpMap.get(), strSettingsFile);

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = std::make_shared<Tracking>(this, mpVocabulary.get(), mpFrameDrawer.get(), mpMapDrawer.get(),
                                               mpMap.get(), mpKeyFrameDatabase.get(), strSettingsFile, mSensor);

        // BAR
        if (continue_mapping)
            mpTracker->InformOnlyTracking(false);

        //Initialize the Local Mapping thread and launch
        mpLocalMapper = std::make_shared<LocalMapping>(mpMap.get(), mSensor == MONOCULAR);
        //mptLocalMapping = std::thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

        //Initialize the Loop Closing thread and launch
        mpLoopCloser = std::make_shared<LoopClosing>(mpMap.get(), mpKeyFrameDatabase.get(), mpVocabulary.get(),
                                                     mSensor != MONOCULAR);
        //mptLoopClosing = std::thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

        //Initialize the Viewer thread and launch
        mpViewer = std::make_shared<Viewer>(this, mpFrameDrawer.get(), mpMapDrawer.get(), mpTracker.get(),
                                            strSettingsFile);
        //Initialize the Viewer thread and launch
        if (bUseViewer)
            mptViewer = std::thread(&Viewer::Run, mpViewer.get());

        //Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper.get());
        mpTracker->SetLoopClosing(mpLoopCloser.get());

        mpLocalMapper->SetTracker(mpTracker.get());
        //mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker.get());
        //mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }

    void System::LoadMap(const std::string &filename) {
        {
            std::cout << filename << std::endl;
            std::ifstream is(filename);

            boost::archive::binary_iarchive ia(is, boost::archive::no_header);
            //ia >> mpKeyFrameDatabase;
            ia >> mpMap;

        }

        // std::ifstream fin("Slam_latest_Map_morning.bin", ios::binary);
        // ostringstream ostrm;

        // ostrm << fin.rdbuf();

        // std::cout << ostrm << std::endl;
        std::cout << std::endl << filename <<" : Map Loaded!" << std::endl;

        std::fstream file(filename, std::ios::binary);
        auto my_str = std::string();
        copy_n(std::istream_iterator<char>(file), 5, std::back_inserter(my_str));
        std::cout << my_str << std::endl;

    }

    void System::SaveMap(const std::string &filename) {
        std::ofstream os(filename);
        {
            std::cout << "Creating file" << std::endl;
            ::boost::archive::binary_oarchive oa(os, ::boost::archive::no_header);
            std::cout << "Writing to file" << std::endl;
            oa << mpMap;
            std::cout << "Wrote to file" << std::endl;
        }
        std::cout << std::endl << "Map saved to " << filename << std::endl;

    }

    cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp) {
        // Check mode change
        {
            //std::unique_lock<std::mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                // mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            //std::unique_lock<std::mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);
        if (Tcw.empty()) {
            return {};
        }
        //std::unique_lock<std::mutex> lock2(mMutexState);
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;

        return Tcw;
    }

    void System::ActivateLocalizationMode() {
        //std::unique_lock<std::mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode() {
        //std::unique_lock<std::mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }


    void System::Reset() {
        //std::unique_lock<std::mutex> lock(mMutexReset);
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
