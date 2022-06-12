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

#include <execution>
#include <utility>
#include "LoopClosing.h"
#include "Sim3Solver.h"
#include "Optimizer.h"


namespace ORB_SLAM2 {

    LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale) :
            mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
            mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(nullptr), mLastLoopKFid(0), mbRunningGBA(false),
            mbStopGBA(false), mpThreadGBA(nullptr), mbFixScale(bFixScale), mnFullBAIdx(false) {
        mnCovisibilityConsistencyTh = 3;
    }

    void LoopClosing::SetTracker(std::shared_ptr<Tracking> pTracker) {
        mpTracker = std::move(pTracker);
    }

    void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper) {
        //mpLocalMapper = pLocalMapper;
    }


    void LoopClosing::Run() {
        mbFinished = false;
        return;
        while (true) {
            // Check if there are keyframes in the queue
            // Detect loop candidates and check covisibility consistency
            if (DetectLoop()) {
                std::cout << "DetectLoop" << std::endl;
                // Compute similarity transformation [sR|t]
                // In the stereo/RGBD case s=1
                if (ComputeSim3()) {
                    std::cout << "ComputeSim3" << std::endl;

                    // Perform loop fusion and pose graph optimization
                    CorrectLoop();
                }
            }


            ResetIfRequested();

            if (CheckFinish())
                break;

            usleep(5000);
        }

        SetFinish();
    }

    void LoopClosing::InsertKeyFrame(KeyFrame *pKF) {
        //std::unique_lock<std::mutex>  lock(mMutexLoopQueue);
        mpCurrentKF = pKF;
        /*if (pKF->mnId != 0)
            mlpLoopKeyFrameQueue.push_back(pKF);*/
    }

    bool LoopClosing::CheckNewKeyFrames() {
        //std::unique_lock<std::mutex>  lock(mMutexLoopQueue);
        return (!mlpLoopKeyFrameQueue.empty());
    }

    bool LoopClosing::DetectLoop() {
        {
            /*if (mlpLoopKeyFrameQueue.empty()) {
                return false;
            }
            //std::unique_lock<std::mutex>  lock(mMutexLoopQueue);
            mpCurrentKF = mlpLoopKeyFrameQueue.front();
            mlpLoopKeyFrameQueue.pop_front();*/
            // Avoid that a keyframe can be erased while it is being process by this thread
            if (!mpCurrentKF) {
                return false;
            }
            mpCurrentKF->SetNotErase();
        }

        //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
        if (mpCurrentKF->mnId < mLastLoopKFid + 10) {
            mpKeyFrameDB->add(mpCurrentKF);
            mpCurrentKF->SetErase();
            return false;
        }

        // Compute reference BoW similarity score
        // This is the lowest score to a connected keyframe in the covisibility graph
        // We will impose loop candidates to have a higher similarity than this
        const std::vector<KeyFrame *> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
        const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
        double minScore = 1;
        for (auto pKF: vpConnectedKeyFrames) {
            if (pKF->isBad())
                continue;
            const DBoW2::BowVector &BowVec = pKF->mBowVec;

            double score = mpORBVocabulary->score(CurrentBowVec, BowVec);

            if (score < minScore)
                minScore = score;
        }

        // Query the database imposing the minimum score
        std::vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

        // If there are no loop candidates, just add new keyframe and return false
        if (vpCandidateKFs.empty()) {
            mpKeyFrameDB->add(mpCurrentKF);
            mvConsistentGroups.clear();
            mpCurrentKF->SetErase();
            return false;
        }

        // For each loop candidate check consistency with previous loop candidates
        // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
        // A group is consistent with a previous group if they share at least a keyframe
        // We must detect a consistent loop in several consecutive keyframes to accept it
        mvpEnoughConsistentCandidates.clear();

        std::vector<ConsistentGroup> vCurrentConsistentGroups;
        std::vector<bool> vbConsistentGroup(mvConsistentGroups.size(), false);
        for (auto pCandidateKF: vpCandidateKFs) {
            std::set<KeyFrame *> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
            spCandidateGroup.insert(pCandidateKF);

            bool bEnoughConsistent = false;
            bool bConsistentForSomeGroup = false;
            for (size_t iG = 0, iendG = mvConsistentGroups.size(); iG < iendG; iG++) {
                std::set<KeyFrame *> sPreviousGroup = mvConsistentGroups[iG].first;

                bool bConsistent = false;
                for (auto sit: spCandidateGroup) {
                    if (sPreviousGroup.count(sit)) {
                        bConsistent = true;
                        bConsistentForSomeGroup = true;
                        break;
                    }
                }

                if (bConsistent) {
                    int nPreviousConsistency = mvConsistentGroups[iG].second;
                    int nCurrentConsistency = nPreviousConsistency + 1;
                    if (!vbConsistentGroup[iG]) {
                        ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
                        vCurrentConsistentGroups.push_back(cg);
                        vbConsistentGroup[iG] = true; //this avoid to include the same group more than once
                    }
                    if (nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent) {
                        mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                        bEnoughConsistent = true; //this avoid to insert the same candidate more than once
                    }
                }
            }

            // If the group is not consistent with any previous group insert with consistency counter set to zero
            if (!bConsistentForSomeGroup) {
                ConsistentGroup cg = make_pair(spCandidateGroup, 0);
                vCurrentConsistentGroups.push_back(cg);
            }
        }

        // Update Covisibility Consistent Groups
        mvConsistentGroups = vCurrentConsistentGroups;


        // Add Current Keyframe to database
        mpKeyFrameDB->add(mpCurrentKF);

        if (mvpEnoughConsistentCandidates.empty()) {
            mpCurrentKF->SetErase();
            return false;
        } else {
            return true;
        }
    }

    bool LoopClosing::ComputeSim3() {
        // For each consistent loop candidate we try to compute a Sim3

        const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

        // We compute first ORB matches for each candidate
        // If enough matches are found, we setup a Sim3Solver
        ORBmatcher matcher(0.75, true);

        std::vector<Sim3Solver *> vpSim3Solvers;
        vpSim3Solvers.resize(nInitialCandidates);

        std::vector<std::vector<std::shared_ptr<MapPoint>>> vvpMapPointMatches;
        vvpMapPointMatches.resize(nInitialCandidates);

        std::vector<bool> vbDiscarded;
        vbDiscarded.resize(nInitialCandidates);

        int nCandidates = 0; //candidates with enough matches

        for (int i = 0; i < nInitialCandidates; i++) {
            KeyFrame *pKF = mvpEnoughConsistentCandidates[i];

            // avoid that local mapping erase it while it is being processed in this thread
            pKF->SetNotErase();

            if (pKF->isBad()) {
                vbDiscarded[i] = true;
                continue;
            }

            int nmatches = matcher.SearchByBoW(mpCurrentKF, pKF, vvpMapPointMatches[i]);

            if (nmatches < 20) {
                vbDiscarded[i] = true;
                continue;
            } else {
                Sim3Solver *pSolver = new Sim3Solver(mpCurrentKF, pKF, vvpMapPointMatches[i], mbFixScale);
                pSolver->SetRansacParameters(0.99, 20, 300);
                vpSim3Solvers[i] = pSolver;
            }

            nCandidates++;
        }

        bool bMatch = false;

        // Perform alternatively RANSAC iterations for each candidate
        // until one is succesful or all fail
        while (nCandidates > 0 && !bMatch) {
            for (int i = 0; i < nInitialCandidates; i++) {
                if (vbDiscarded[i])
                    continue;

                KeyFrame *pKF = mvpEnoughConsistentCandidates[i];

                // Perform 5 Ransac Iterations
                std::vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                Sim3Solver *pSolver = vpSim3Solvers[i];
                cv::Mat Scm = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore) {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
                if (!Scm.empty()) {
                    std::vector<std::shared_ptr<MapPoint>> vpMapPointMatches(vvpMapPointMatches[i].size(),
                                                                             nullptr);
                    for (size_t j = 0, jend = vbInliers.size(); j < jend; j++) {
                        if (vbInliers[j])
                            vpMapPointMatches[j] = vvpMapPointMatches[i][j];
                    }

                    cv::Mat R = pSolver->GetEstimatedRotation();
                    cv::Mat t = pSolver->GetEstimatedTranslation();
                    const float s = pSolver->GetEstimatedScale();
                    ORB_SLAM2::ORBmatcher::SearchBySim3(mpCurrentKF, pKF, vpMapPointMatches, s, R, t, 7.5);

                    g2o::Sim3 gScm(Converter::toMatrix3d(R), Converter::toVector3d(t), s);
                    const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10,
                                                                 mbFixScale);

                    // If optimization is succesful stop ransacs and continue
                    if (nInliers >= 20) {
                        bMatch = true;
                        mpMatchedKF = pKF;
                        g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),
                                       Converter::toVector3d(pKF->GetTranslation()), 1.0);
                        mg2oScw = gScm * gSmw;
                        mScw = Converter::toCvMat(mg2oScw);

                        mvpCurrentMatchedPoints = vpMapPointMatches;
                        break;
                    }
                }
            }
        }

        if (!bMatch) {
            for (int i = 0; i < nInitialCandidates; i++)
                mvpEnoughConsistentCandidates[i]->SetErase();
            mpCurrentKF->SetErase();
            return false;
        }

        // Retrieve MapPoints seen in Loop Keyframe and neighbors
        std::vector<KeyFrame *> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
        vpLoopConnectedKFs.push_back(mpMatchedKF);
        mvpLoopMapPoints.clear();
        for (auto pKF: vpLoopConnectedKFs) {
            auto vpMapPoints = pKF->GetMapPointMatches();
            for (auto &[i, pMP]: vpMapPoints) {
                if (pMP) {
                    if (!pMP->isBad() && pMP->mnLoopPointForKF != mpCurrentKF->mnId) {
                        mvpLoopMapPoints.push_back(pMP);
                        pMP->mnLoopPointForKF = mpCurrentKF->mnId;
                    }
                }
            }
        }

        // Find more matches projecting with the computed Sim3
        ORB_SLAM2::ORBmatcher::SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints, 10);

        // If enough matches accept Loop
        int nTotalMatches = 0;
        for (auto &mvpCurrentMatchedPoint: mvpCurrentMatchedPoints) {
            if (mvpCurrentMatchedPoint)
                nTotalMatches++;
        }

        if (nTotalMatches >= 40) {
            for (int i = 0; i < nInitialCandidates; i++)
                if (mvpEnoughConsistentCandidates[i] != mpMatchedKF)
                    mvpEnoughConsistentCandidates[i]->SetErase();
            return true;
        } else {
            for (int i = 0; i < nInitialCandidates; i++)
                mvpEnoughConsistentCandidates[i]->SetErase();
            mpCurrentKF->SetErase();
            return false;
        }

    }


    void LoopClosing::CorrectLoop() {
        std::cout << "Loop detected!" << std::endl;

        // Send a stop signal to Local Mapping
        // Avoid new keyframes are inserted while correcting the loop
        //mpLocalMapper->RequestStop();

        // If a Global Bundle Adjustment is running, abort it
        if (isRunningGBA()) {
            //std::unique_lock<std::mutex>  lock(mMutexGBA);
            mbStopGBA = true;

            mnFullBAIdx = true;

            if (mpThreadGBA) {
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
        }
        std::cout << "bundle adjustment aborted" << std::endl;
        // Wait until Local Mapping has effectively stopped
        /*while (!mpLocalMapper->isStopped()) {
            usleep(100);
        }*/
        std::cout << "local map stopped" << std::endl;
        // Ensure current keyframe is updated
        mpCurrentKF->UpdateConnections();

        // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
        mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
        mvpCurrentConnectedKFs.push_back(mpCurrentKF);

        KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
        CorrectedSim3[mpCurrentKF] = mg2oScw;
        cv::Mat Twc = mpCurrentKF->GetPoseInverse();


        {
            // Get Map Mutex
            //std::unique_lock<std::mutex>  lock(mpMap->mMutexMapUpdate);
            for (auto pKFi: mvpCurrentConnectedKFs) {
                if (pKFi) {
                    cv::Mat Tiw = pKFi->GetPose();
                    if (pKFi != mpCurrentKF) {
                        cv::Mat Tic = Tiw * Twc;
                        cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
                        cv::Mat tic = Tic.rowRange(0, 3).col(3);
                        g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric), Converter::toVector3d(tic), 1.0);
                        g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oScw;
                        //Pose corrected with the Sim3 of the loop closure
                        CorrectedSim3[pKFi] = g2oCorrectedSiw;
                    }
                    cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
                    cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
                    g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);
                    //Pose without correction
                    NonCorrectedSim3[pKFi] = g2oSiw;
                }
            }
            // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
            for (auto pKFi: CorrectedSim3) {
                if (pKFi.first) {
                    g2o::Sim3 g2oCorrectedSiw = pKFi.second;
                    g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

                    g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi.first];

                    for (auto &[i, pMPi]: pKFi.first->GetMapPointMatches()) {
                        if (!pMPi || pMPi->isBad() || pMPi->mnCorrectedByKF == mpCurrentKF->mnId)
                            continue;

                        // Project with non-corrected pose and project back with corrected pose
                        pMPi->SetWorldPos(Converter::toCvMat(
                                g2oCorrectedSwi.map(g2oSiw.map(Converter::toVector3d(pMPi->GetWorldPos())))));
                        pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                        pMPi->mnCorrectedReference = pKFi.first->mnId;
                        pMPi->UpdateNormalAndDepth();
                    }
                    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
                    double s = g2oCorrectedSiw.scale();
                    eigt *= (1. / s); //[R t/s;0 1]
                    pKFi.first->SetPose(Converter::toCvSE3(g2oCorrectedSiw.rotation().toRotationMatrix(), eigt));
                    // Make sure connections are updated
                    pKFi.first->UpdateConnections();
                }
            }
            // Start Loop Fusion
            // Update matched map points and replace if duplicated
            for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++) {
                if (mvpCurrentMatchedPoints[i]) {
                    auto pLoopMP = mvpCurrentMatchedPoints[i];
                    auto pCurMP = mpCurrentKF->GetMapPoint(i);
                    if (pCurMP)
                        pCurMP->Replace(pLoopMP);
                    else {
                        mpCurrentKF->AddMapPoint(pLoopMP, i);
                        pLoopMP->AddObservation(mpCurrentKF, i);
                        pLoopMP->ComputeDistinctiveDescriptors();
                    }
                }
            }

        }

        // Project MapPoints observed in the neighborhood of the loop keyframe
        // into the current keyframe and neighbors using corrected poses.
        // Fuse duplications.
        SearchAndFuse(CorrectedSim3);


        // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
        std::map<KeyFrame *, std::set<KeyFrame *> > LoopConnections;
        for (auto pKFi: mvpCurrentConnectedKFs) {
            if (pKFi) {
                // Update connections. Detect new links.
                pKFi->UpdateConnections();
                LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
                for (auto prev: pKFi->GetVectorCovisibleKeyFrames()) {
                    LoopConnections[pKFi].erase(prev);
                }
                for (auto prev: mvpCurrentConnectedKFs) {
                    LoopConnections[pKFi].erase(prev);
                }
            }
        }
        // Optimize graph
        Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3,
                                          LoopConnections, mbFixScale);

        mpMap->InformNewBigChange();

        // Add loop edge
        mpMatchedKF->AddLoopEdge(mpCurrentKF);
        mpCurrentKF->AddLoopEdge(mpMatchedKF);

        // Launch a new thread to perform Global Bundle Adjustment
        mbRunningGBA = true;
        mbStopGBA = false;
        mpThreadGBA = new std::thread(&LoopClosing::RunGlobalBundleAdjustment, this, mpCurrentKF->mnId);

        // Loop closed. Release Local Mapping.
        //mpLocalMapper->Release();

        mLastLoopKFid = mpCurrentKF->mnId;
        std::cout << "done loop closing" << std::endl;
    }

    void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap) {
        ORBmatcher matcher(0.8);
        for (auto pKF: CorrectedPosesMap) {
            g2o::Sim3 g2oScw = pKF.second;
            cv::Mat cvScw = Converter::toCvMat(g2oScw);

            std::vector<std::shared_ptr<MapPoint>> vpReplacePoints(mvpLoopMapPoints.size(), nullptr);
            ORB_SLAM2::ORBmatcher::Fuse(pKF.first, cvScw, mvpLoopMapPoints, 4, vpReplacePoints);
            //std::unique_lock<std::mutex>  lock(mpMap->mMutexMapUpdate);
            const int nLP = mvpLoopMapPoints.size();
            for (int i = 0; i < nLP; i++) {
                auto pRep = vpReplacePoints[i];
                if (pRep) {
                    pRep->Replace(mvpLoopMapPoints[i]);
                }
            }
        }
    }


    void LoopClosing::RequestReset() {
        {
            //std::unique_lock<std::mutex>  lock(mMutexReset);
            mbResetRequested = true;
        }

        while (true) {
            {
                //std::unique_lock<std::mutex>  lock2(mMutexReset);
                if (!mbResetRequested)
                    break;
            }
            usleep(5000);
        }
    }

    void LoopClosing::ResetIfRequested() {
        //std::unique_lock<std::mutex>  lock(mMutexReset);
        if (mbResetRequested) {
            mlpLoopKeyFrameQueue.clear();
            mLastLoopKFid = 0;
            mbResetRequested = false;
        }
    }

    void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF) {
        int idx = mnFullBAIdx;
        Optimizer::GlobalBundleAdjustemnt(mpMap, 10, &mbStopGBA, nLoopKF, true);

        // Update all MapPoints and KeyFrames
        // Local Mapping was active during BA, that means that there might be new keyframes
        // not included in the Global BA and they are not consistent with the updated map.
        // We need to propagate the correction through the spanning tree
        {
            //std::unique_lock<std::mutex>  lock(mMutexGBA);
            if (idx != mnFullBAIdx)
                return;

            if (!mbStopGBA) {
                //mpLocalMapper->RequestStop();
                // Wait until Local Mapping has effectively stopped

                /*while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished()) {
                    usleep(1000);
                }*/

                // Get Map Mutex
                //std::unique_lock<std::mutex>  lock(mpMap->mMutexMapUpdate);

                // Correct keyframes starting at map first keyframe
                std::list<KeyFrame *> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(), mpMap->mvpKeyFrameOrigins.end());

                while (!lpKFtoCheck.empty()) {
                    KeyFrame *pKF = lpKFtoCheck.front();
                    const std::set<KeyFrame *> sChilds = pKF->GetChilds();
                    cv::Mat Twc = pKF->GetPoseInverse();
                    for (auto pChild: sChilds) {
                        if (pChild->mnBAGlobalForKF != nLoopKF) {
                            cv::Mat Tchildc = pChild->GetPose() * Twc;
                            pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                            pChild->mnBAGlobalForKF = nLoopKF;
                        }
                        lpKFtoCheck.push_back(pChild);
                    }

                    pKF->mTcwBefGBA = pKF->GetPose();
                    pKF->SetPose(pKF->mTcwGBA);
                    lpKFtoCheck.pop_front();
                }

                // Correct MapPoints
                const auto vpMPs = mpMap->GetAllMapPoints();

                for (auto pMP: vpMPs) {
                    if (pMP->isBad())
                        continue;

                    if (pMP->mnBAGlobalForKF == nLoopKF) {
                        // If optimized by Global BA, just update
                        pMP->SetWorldPos(pMP->mPosGBA);
                    } else {
                        // Update according to the correction of its reference keyframe
                        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

                        if (pRefKF->mnBAGlobalForKF != nLoopKF)
                            continue;

                        // Map to non-corrected camera
                        cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0, 3).colRange(0, 3);
                        cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0, 3).col(3);
                        cv::Mat Xc = Rcw * pMP->GetWorldPos() + tcw;

                        // Backproject using corrected camera
                        cv::Mat Twc = pRefKF->GetPoseInverse();
                        cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
                        cv::Mat twc = Twc.rowRange(0, 3).col(3);

                        pMP->SetWorldPos(Rwc * Xc + twc);
                    }
                }

                mpMap->InformNewBigChange();

                //mpLocalMapper->Release();

            }

            mbFinished = true;
            mbRunningGBA = false;
        }
    }

    void LoopClosing::RequestFinish() {
        //std::unique_lock<std::mutex>  lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool LoopClosing::CheckFinish() {
        //std::unique_lock<std::mutex>  lock(mMutexFinish);
        return mbFinishRequested;
    }

    void LoopClosing::SetFinish() {
        //std::unique_lock<std::mutex>  lock(mMutexFinish);
        mbFinished = true;
    }

    bool LoopClosing::isFinished() {
        //std::unique_lock<std::mutex>  lock(mMutexFinish);
        return mbFinished;
    }


} //namespace ORB_SLAM
