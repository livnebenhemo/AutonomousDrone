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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2 {

    Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0) {
    }

    template<class Archive>
    void Map::serialize(Archive &ar, const unsigned int version) {
        // don't save mutex
        ar & mspMapPoints;
        ar & mvpKeyFrameOrigins;
        ar & mspKeyFrames;
        ar & mvpReferenceMapPoints;
        ar & mnMaxKFid & mnBigChangeIdx;
    }

    template void Map::serialize(boost::archive::binary_iarchive &, const unsigned int);

    template void Map::serialize(boost::archive::binary_oarchive &, const unsigned int);

    void Map::AddKeyFrame(KeyFrame *pKF) {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        if (!mspKeyFrames.count(pKF)) {
            mspKeyFrames.insert({pKF, pKF->mnId});

            if (pKF->mnId > mnMaxKFid)
                mnMaxKFid = pKF->mnId;
        }
    }

    void Map::AddMapPoint(const std::shared_ptr<MapPoint>&pMP) {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        if (!mspMapPoints.count(pMP->mnId)) {
            mspMapPoints[pMP->mnId] = pMP;
        }
    }

    void Map::EraseMapPoint(const std::shared_ptr<MapPoint>&pMP) {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        if (pMP)
        mspMapPoints.erase(pMP->mnId);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::EraseKeyFrame(KeyFrame *pKF) {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::SetReferenceMapPoints(const std::vector<std::shared_ptr<MapPoint>> &vpMPs) {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange() {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    int Map::GetLastBigChangeIdx() {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    std::vector<KeyFrame *> Map::GetAllKeyFrames() {
        // std::unique_lock<std::mutex> lock(mMutexMap);
        std::vector<KeyFrame *> keyFrames;
        for (const auto &keyFrame: mspKeyFrames) {
            keyFrames.emplace_back(keyFrame.first);
        }
        return keyFrames;
    }

    std::vector<std::shared_ptr<MapPoint>> Map::GetAllMapPoints() {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        std::vector<std::shared_ptr<MapPoint>> mapPoints;
        for (const auto &mapPoint: mspMapPoints) {
            mapPoints.emplace_back(mapPoint.second);
        }
        return mapPoints;
    }

    long unsigned int Map::MapPointsInMap() {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap() {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    std::vector<std::shared_ptr<MapPoint>> Map::GetReferenceMapPoints() {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetMaxKFid() {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void Map::clear() {
        /*for (auto &mspMapPoint: mspMapPoints) {
            delete mspMapPoint.second;
        }
        for (auto &mspMapPoint: mspCurrentMapPoints) {
            delete mspMapPoint;
        }*/
        for (auto mspKeyFrame: mspKeyFrames) {
            delete mspKeyFrame.first;
        }
        for (auto mspKeyFrame: mspCurrentKeyFrames) {
            delete mspKeyFrame;
        }
        if (!mspMapPoints.empty()) {
            mspMapPoints.clear();
        }
        if (!mspKeyFrames.empty()) {
            mspKeyFrames.clear();

        }
        mnMaxKFid = 0;
        /*for (auto mvpReferenceMapPoint: mvpReferenceMapPoints){
            std::cout << mvpReferenceMapPoint <<std::endl;
            delete mvpReferenceMapPoint;
        }


        for (auto mvpKeyFrameOrigin: mvpKeyFrameOrigins){
            std::cout << mvpKeyFrameOrigin <<std::endl;
            delete mvpKeyFrameOrigin;
        }*/
        mvpReferenceMapPoints.clear();
        mvpReferenceMapPoints.clear();
    }

} //namespace ORB_SLAM
