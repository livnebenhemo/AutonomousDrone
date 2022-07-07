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

#include "MapPoint.h"

namespace ORB_SLAM2 {

    long unsigned int MapPoint::nNextId = 0;
    std::mutex MapPoint::mGlobalMutex;

    MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap) :
            mnFirstKFid(pRefKF->mnId), nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<std::shared_ptr<MapPoint>>(nullptr)), mfMinDistance(0), mfMaxDistance(0),
            mpMap(pMap) {
        Pos.copyTo(mWorldPos);
        mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
    }

    MapPoint::MapPoint() :
            nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<std::shared_ptr<MapPoint>>(NULL)), mfMinDistance(0), mfMaxDistance(0) {
        //mNormalVector = cv::Mat::zeros(3,1,CV_32F);
        // std::unique_lock<recursive_mutex> lock(mpMap->mMutexPointCreation);
        //mpMap = new Map();
        // mpRefKF = new KeyFrame();
    }

    MapPoint::MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF) :
            mnFirstKFid(-1), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
            mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame *>(nullptr)), mnVisible(1),
            mnFound(1), mbBad(false), mpReplaced(nullptr), mpMap(pMap) {
        Pos.copyTo(mWorldPos);
        cv::Mat Ow = pFrame->GetCameraCenter();
        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / cv::norm(mNormalVector);

        cv::Mat PC = Pos - Ow;
        const double dist = cv::norm(PC);
        const int level = pFrame->mvKeysUn[idxF].octave;
        const double levelScaleFactor = pFrame->mvScaleFactors[level];
        const int nLevels = pFrame->mnScaleLevels;

        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

        pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
    }

    template<class Archive>
    void MapPoint::serialize(Archive &ar, const unsigned int version) {
        ar & mnId & nNextId & mnFirstKFid & nObs;
        // Tracking related vars
        ar & mTrackProjX;
        ar & mTrackProjY;
        ar & mTrackProjXR;
        ar & mbTrackInView;
        ar & mnTrackScaleLevel;
        ar & mTrackViewCos;
        ar & mnTrackReferenceForFrame;
        ar & mnLastFrameSeen;
        // Local Mapping related vars
        ar & mnBALocalForKF & mnFuseCandidateForKF;
        // Loop Closing related vars
        ar & mnLoopPointForKF & mnCorrectedByKF & mnCorrectedReference & mPosGBA & mnBAGlobalForKF;
        // don't save the mutex
        ar & mWorldPos;
        ar & mObservations;
        ar & mNormalVector;
        ar & mDescriptor;
        ar & mpRefKF;
        ar & mnVisible & mnFound;
        ar & mbBad & mpReplaced;
        ar & mfMinDistance & mfMaxDistance;
        ar & mpMap;
        // don't save the mutex
    }

    template void MapPoint::serialize(boost::archive::binary_iarchive &, const unsigned int);

    template void MapPoint::serialize(boost::archive::binary_oarchive &, const unsigned int);

    void MapPoint::SetMap(const std::shared_ptr<Map> &map) {
        mpMap = map.get();//std::move(map);
    }

    void MapPoint::SetObservations(std::vector<KeyFrame *> spKeyFrames) {

        long unsigned int id, kfRef_id;
        size_t size;
        // std::cout << "KF" << mnId <<" valid indexes-" <<  std::endl;
        int j = 0;
        bool found_reference = false;
        kfRef_id = mref_KfId_pair.first;
        bool is_ref_valid = mref_KfId_pair.second;


        for (auto it = mObservations_nId.begin();
             it != mObservations_nId.end(); j++, ++it) {
            id = it->first;
            size = it->second;
            {
                for (std::vector<KeyFrame *>::iterator mit = spKeyFrames.begin(); mit != spKeyFrames.end(); mit++) {
                    KeyFrame *pKf = *mit;
                    // std::cout << "[" << pKf->mnId << "]";
                    if (id == pKf->mnId) {
                        // std::cout << "[" << id <<"]";
                        mObservations[pKf] = size;
                        //id = -1;
                        break;
                    }
                }

            }

        }

        for (std::vector<KeyFrame *>::iterator mit = spKeyFrames.begin(); mit != spKeyFrames.end(); mit++) {
            KeyFrame *pKf = *mit;
            if (is_ref_valid && kfRef_id == pKf->mnId) {
                // Set the refernce Keyframe
                mpRefKF = pKf;
                found_reference = true;
            }
        }

        if (!found_reference) {
            mpRefKF = static_cast<KeyFrame *>(NULL);
            // std::cout << "refernce KF - " << kfRef_id << "is not found for mappoint " << mnId <<  std::endl;
            // Dummy KF
            //mpRefKF = new KeyFrame();
        }
    }

    void MapPoint::SetWorldPos(const cv::Mat &Pos) {
        std::unique_lock<std::mutex> lock2(mGlobalMutex);
        std::unique_lock<std::mutex> lock(mMutexPos);
        Pos.copyTo(mWorldPos);
    }

    cv::Mat MapPoint::GetWorldPos() {
        std::unique_lock<std::mutex> lock(mMutexPos);
        return mWorldPos.clone();
    }

    cv::Mat MapPoint::GetNormal() {
        std::unique_lock<std::mutex> lock(mMutexPos);
        return mNormalVector.clone();
    }

    KeyFrame *MapPoint::GetReferenceKeyFrame() {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    void MapPoint::AddObservation(KeyFrame *pKF, size_t idx) {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return;
        mObservations[pKF] = idx;

        if (pKF->mvuRight[idx] >= 0)
            nObs += 2;
        else
            nObs++;
    }

    void MapPoint::EraseObservation(KeyFrame *pKF) {
        bool bBad = false;
        {
            //std::unique_lock<std::mutex> lock(mMutexFeatures);
            if (pKF && mObservations.erase(pKF)) {
                nObs--;
                if (mpRefKF == pKF && !mObservations.empty() && mObservations.begin()->first)
                    mpRefKF = mObservations.begin()->first;
                // If only 2 observations or less, discard point
                bBad = nObs <= 2;
            }
        }
        if (bBad)
            SetBadFlag();
    }

    std::unordered_map<KeyFrame *, size_t> MapPoint::GetObservations() {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        return mObservations;
    }

    int MapPoint::Observations() {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        return nObs;
    }

    void MapPoint::SetBadFlag() {
        //std::unordered_map<KeyFrame *, size_t> obs;
        /*{
            std::unique_lock<std::mutex> lock1(mMutexFeatures);
            //std::unique_lock<std::mutex> lock2(mMutexPos);
            mbBad = true;
            obs = mObservations;
            mObservations.clear();
        }*/
        if (!mObservations.empty()) {
            for (auto pKf: mObservations) {
                pKf.first->EraseMapPointMatch(pKf.second);
            }
            mObservations.clear();
        }
        mbBad = true;

    }

    std::shared_ptr<MapPoint> MapPoint::GetReplaced() {
        //std::unique_lock<std::mutex> lock1(mMutexFeatures);
        //std::unique_lock<std::mutex> lock2(mMutexPos);
        return mpReplaced;
    }

    void MapPoint::Replace(std::shared_ptr<MapPoint> pMP) {
        if (!pMP || pMP->mnId == this->mnId)
            return;

        int nvisible, nfound;
        std::unordered_map<KeyFrame *, size_t> obs;
        {
            // std::unique_lock<std::mutex> lock1(mMutexFeatures);
            // std::unique_lock<std::mutex> lock2(mMutexPos);
            obs = mObservations;
            mObservations.clear();
            mbBad = true;
            nvisible = mnVisible;
            nfound = mnFound;
            mpReplaced = pMP;
        }
        for (auto pKF: obs) {
            if (!pMP->IsInKeyFrame(pKF.first)) {
                pKF.first->ReplaceMapPointMatch(pKF.second, pMP);
                pMP->AddObservation(pKF.first, pKF.second);
            } else {
                pKF.first->EraseMapPointMatch(pKF.second);
            }
        }
        pMP->IncreaseFound(nfound);
        pMP->IncreaseVisible(nvisible);
        pMP->ComputeDistinctiveDescriptors();
    }

    bool MapPoint::isBad() {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        //std::unique_lock<std::mutex> lock2(mMutexPos);
        return mbBad;
    }

    void MapPoint::IncreaseVisible(int n) {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        mnVisible += n;
    }

    void MapPoint::IncreaseFound(int n) {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        mnFound += n;
    }

    float MapPoint::GetFoundRatio() {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound) / static_cast<float>(mnVisible);
    }

    void MapPoint::ComputeDistinctiveDescriptors() {
        // Retrieve all observed descriptors
        std::vector<cv::Mat> vDescriptors;

        std::unordered_map<KeyFrame *, size_t> observations;

        {
            //std::unique_lock<std::mutex> lock1(mMutexFeatures);
            if (mbBad)
                return;
            observations = mObservations;
        }

        if (observations.empty())
            return;

        vDescriptors.reserve(observations.size());
        for (auto observation: observations) {
            if (!observation.first->isBad()) {
                vDescriptors.push_back(observation.first->mDescriptors.row(observation.second));
            }
        }
        if (vDescriptors.empty())
            return;

        // Compute distances between them
        const size_t N = vDescriptors.size();

        double Distances[N][N];
        for (size_t i = 0; i < N; i++) {
            Distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++) {
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
                Distances[i][j] = distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for (size_t i = 0; i < N; i++) {
            std::vector<int> vDists(Distances[i], Distances[i] + N);
            sort(vDists.begin(), vDists.end());
            int median = vDists[0.5 * (N - 1)];

            if (median < BestMedian) {
                BestMedian = median;
                BestIdx = i;
            }
        }

        {
            // std::unique_lock<std::mutex> lock(mMutexFeatures);
            mDescriptor = vDescriptors[BestIdx].clone();
        }
    }

    cv::Mat MapPoint::GetDescriptor() {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }

    int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF) {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    bool MapPoint::IsInKeyFrame(KeyFrame *pKF) {
        // std::unique_lock<std::mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    void MapPoint::UpdateNormalAndDepth() {
        std::unordered_map<KeyFrame *, size_t> observations;
        KeyFrame *pRefKF;
        cv::Mat Pos;
        if (mObservations.empty() || !mpRefKF)
            return;
        {
            //std::unique_lock<std::mutex> lock1(mMutexFeatures);
            //std::unique_lock<std::mutex> lock2(mMutexPos);
            if (mbBad)
                return;
            observations = mObservations;
            pRefKF = mpRefKF;
            Pos = mWorldPos.clone();
        }

        cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
        auto n = observations.size();
        for (auto observation: observations) {
            auto pKF = observation.first;
            cv::Mat normali = Pos - pKF->GetCameraCenter();
            normal += normali / cv::norm(normali);
        }

        cv::Mat PC = Pos - pRefKF->GetCameraCenter();
        const double dist = cv::norm(PC);
        const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
        const double levelScaleFactor = pRefKF->mvScaleFactors[level];
        const int nLevels = pRefKF->mnScaleLevels;

        {
            // std::unique_lock<std::mutex> lock3(mMutexPos);
            mfMaxDistance = dist * levelScaleFactor;
            mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
            mNormalVector = normal / n;
        }
    }

    float MapPoint::GetMinDistanceInvariance() {
        std::unique_lock<std::mutex> lock(mMutexPos);
        return 0.8f * mfMinDistance;
    }

    float MapPoint::GetMaxDistanceInvariance() {
        //std::unique_lock<std::mutex> lock(mMutexPos);
        return 1.2f * mfMaxDistance;
    }

    int MapPoint::PredictScale(const double &currentDist, KeyFrame *pKF) {
        double ratio;
        {
            //std::unique_lock<std::mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pKF->mnScaleLevels)
            nScale = pKF->mnScaleLevels - 1;

        return nScale;
    }

    int MapPoint::PredictScale(const double &currentDist, Frame *pF) {
        double ratio;
        {
            //std::unique_lock<std::mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pF->mnScaleLevels)
            nScale = pF->mnScaleLevels - 1;

        return nScale;
    }


} //namespace ORB_SLAM
