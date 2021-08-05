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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2 {

    KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary &voc) :
            mpVoc(&voc) {
        mvInvertedFile.resize(voc.size());
    }


    void KeyFrameDatabase::add(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutex);
        for (auto vit : pKF->mBowVec) {
            mvInvertedFile[vit.first].push_back(pKF);
        }
    }

    void KeyFrameDatabase::erase(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutex);

        // Erase elements in the Inverse File for the entry
        for (auto vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end();
             vit != vend; vit++) {
            // List of keyframes that share the word
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

            for (auto lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
                if (pKF == *lit) {
                    lKFs.erase(lit);
                    break;
                }
            }
        }
    }

    void KeyFrameDatabase::clear() {
        mvInvertedFile.clear();
        mvInvertedFile.resize(mpVoc->size());
    }


    vector<KeyFrame *> KeyFrameDatabase::DetectLoopCandidates(KeyFrame *pKF, double minScore) {
        //TODO: change to dic
        auto spConnectedKeyFrames = pKF->GetConnectedKeyFramesAsDic();
        list<KeyFrame *> lKFsSharingWords;
        // Search all keyframes that share a word with current keyframes
        //TODO:change to check all
        // Discard keyframes connected to the query keyframe
        {
            unique_lock<mutex> lock(mMutex);
            for (auto bowVector : pKF->mBowVec) {
                list<KeyFrame *> lKFs = mvInvertedFile[bowVector.first];
                for (auto pKFi : lKFs) {
                    if (pKFi->mnLoopQuery != pKF->mnId) {
                        pKFi->mnLoopWords = 0;
                        if (!spConnectedKeyFrames.count(pKFi)) {
                            pKFi->mnLoopQuery = pKF->mnId;
                            lKFsSharingWords.push_back(pKFi);
                        }
                    }
                    pKFi->mnLoopWords++;
                }
            }

            if (lKFsSharingWords.empty())
                return vector<KeyFrame *>();

            list<pair<float, KeyFrame *> > lScoreAndMatch;
            //TODO: change magic number

            // Compute similarity score. Retain the matches whose score is higher than minScore
            for (auto pKFi : lKFsSharingWords) {
                double si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

                pKFi->mLoopScore = si;
                if (si >= minScore)
                    lScoreAndMatch.emplace_back(make_pair(si, pKFi));
            }

            if (lScoreAndMatch.empty())
                return vector<KeyFrame *>();
            // Lets now accumulate score by covisibility
            list<pair<float, KeyFrame *> > lAccScoreAndMatch;
            double bestAccScore = minScore;
            for (auto scoreAndMatch : lScoreAndMatch) {
                KeyFrame *pKFi = scoreAndMatch.second;
                //TODO: is the CovisibilityKeyFrames ordered
                vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);
                double bestScore = scoreAndMatch.first, accScore = scoreAndMatch.first;
                KeyFrame *pBestKF = pKFi;
                for (auto pKF2 : vpNeighs) {
                    if (pKF2->mnLoopQuery == pKF->mnId) {
                        accScore += pKF2->mLoopScore;
                        if (pKF2->mLoopScore > bestScore) {
                            pBestKF = pKF2;
                            bestScore = pKF2->mLoopScore;
                        }
                    }
                }

                lAccScoreAndMatch.emplace_back(make_pair(accScore, pBestKF));
                bestAccScore = accScore > bestAccScore ? accScore : bestAccScore;
            }
            // Return all those keyframes with a score higher than 0.75*bestScore
            float minScoreToRetain = 0.75f * bestAccScore;
            vector<KeyFrame *> vpLoopCandidates;
            std::unordered_map<KeyFrame *, int> spAlreadyAddedKF{};
            spAlreadyAddedKF.reserve(lAccScoreAndMatch.size());
            for (auto accScoreAndMatch : lAccScoreAndMatch) {
                if (accScoreAndMatch.first > minScoreToRetain) {
                    KeyFrame *pKFi = accScoreAndMatch.second;
                    if (!spAlreadyAddedKF.count(pKFi)) {
                        spAlreadyAddedKF.insert({pKFi, 0});
                        vpLoopCandidates.emplace_back(pKFi);
                    }
                }
            }
            return vpLoopCandidates;
        }
    }

    vector<KeyFrame *> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F) {
        list<KeyFrame*> lKFsSharingWords;

        // Search all keyframes that share a word with current frame
        {
            unique_lock<mutex> lock(mMutex);

            for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
            {
                list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

                for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
                {
                    KeyFrame* pKFi=*lit;
                    if(pKFi->mnRelocQuery!=F->mnId)
                    {
                        pKFi->mnRelocWords=0;
                        pKFi->mnRelocQuery=F->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                    pKFi->mnRelocWords++;
                }
            }
        }
        if(lKFsSharingWords.empty())
            return vector<KeyFrame*>();

        // Only compare against those keyframes that share enough words
        int maxCommonWords=0;
        for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
        {
            if((*lit)->mnRelocWords>maxCommonWords)
                maxCommonWords=(*lit)->mnRelocWords;
        }

        int minCommonWords = maxCommonWords*0.6f;

        list<pair<float,KeyFrame*> > lScoreAndMatch;

        int nscores=0;

        // Compute similarity score.
        for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
        {
            KeyFrame* pKFi = *lit;

            if(pKFi->mnRelocWords>minCommonWords)
            {
                nscores++;
                float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
                pKFi->mRelocScore=si;
                lScoreAndMatch.push_back(make_pair(si,pKFi));
            }
        }

        if(lScoreAndMatch.empty())
            return vector<KeyFrame*>();

        list<pair<float,KeyFrame*> > lAccScoreAndMatch;
        float bestAccScore = 0;

        // Lets now accumulate score by covisibility
        for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
        {
            KeyFrame* pKFi = it->second;
            vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(20);

            float bestScore = it->first;
            float accScore = bestScore;
            KeyFrame* pBestKF = pKFi;
            for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
            {
                KeyFrame* pKF2 = *vit;
                if(pKF2->mnRelocQuery!=F->mnId)
                    continue;

                accScore+=pKF2->mRelocScore;
                if(pKF2->mRelocScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mRelocScore;
                }

            }
            lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
            if(accScore>bestAccScore)
                bestAccScore=accScore;
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        float minScoreToRetain = 0.6f*bestAccScore;
        set<KeyFrame*> spAlreadyAddedKF;
        vector<KeyFrame*> vpRelocCandidates;
        vpRelocCandidates.reserve(lAccScoreAndMatch.size());
        for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
        {
            const float &si = it->first;
            if(si>minScoreToRetain)
            {
                KeyFrame* pKFi = it->second;
                if(!spAlreadyAddedKF.count(pKFi))
                {
                    vpRelocCandidates.push_back(pKFi);
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        }
        return vpRelocCandidates;
    }

} //namespace ORB_SLAM
