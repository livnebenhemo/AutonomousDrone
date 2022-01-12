

#include "include/KeyFrameDatabase.h"

#include "include/KeyFrame.h"
#include "../Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>


namespace ORB_SLAM2 {

    KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary &voc) :
            mpVoc(&voc) {
        mvInvertedFile.resize(voc.size());
    }


    void KeyFrameDatabase::add(KeyFrame *pKF) {
        std::unique_lock<std::mutex> lock(mMutex);

        for (auto vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            mvInvertedFile[vit->first].push_back(pKF);
    }

    void KeyFrameDatabase::erase(KeyFrame *pKF) {
        std::unique_lock<std::mutex> lock(mMutex);

        // Erase elements in the Inverse File for the entry
        for (auto vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++) {
            // List of keyframes that share the word
            std::list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

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


    std::vector<KeyFrame *> KeyFrameDatabase::DetectLoopCandidates(KeyFrame *pKF, double minScore) {
        std::set<KeyFrame *> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
        std::list<KeyFrame *> lKFsSharingWords;

        // Search all keyframes that share a word with current keyframes
        // Discard keyframes connected to the query keyframe
        {
            std::unique_lock<std::mutex> lock(mMutex);

            for (auto vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++) {
                std::list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                for (auto pKFi: lKFs) {
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
        }

        if (lKFsSharingWords.empty())
            return {};

        std::list<std::pair<float, KeyFrame *> > lScoreAndMatch;

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (auto &lKFsSharingWord: lKFsSharingWords) {
            if (lKFsSharingWord->mnLoopWords > maxCommonWords)
                maxCommonWords = lKFsSharingWord->mnLoopWords;
        }

        int minCommonWords = std::ceil(maxCommonWords * 0.8);

        int nscores = 0;

        // Compute similarity score. Retain the matches whose score is higher than minScore
        for (auto pKFi: lKFsSharingWords) {
            if (pKFi->mnLoopWords > minCommonWords) {
                nscores++;

                double si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

                pKFi->mLoopScore = si;
                if (si >= minScore)
                    lScoreAndMatch.emplace_back(si, pKFi);
            }
        }

        if (lScoreAndMatch.empty())
            return {};

        std::list<std::pair<float, KeyFrame *> > lAccScoreAndMatch;
        double bestAccScore = minScore;

        // Lets now accumulate score by covisibility
        for (auto &it: lScoreAndMatch) {
            KeyFrame *pKFi = it.second;
            std::vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            double bestScore = it.first;
            double accScore = it.first;
            KeyFrame *pBestKF = pKFi;
            for (auto pKF2: vpNeighs) {
                if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords) {
                    accScore += pKF2->mLoopScore;
                    if (pKF2->mLoopScore > bestScore) {
                        pBestKF = pKF2;
                        bestScore = pKF2->mLoopScore;
                    }
                }
            }

            lAccScoreAndMatch.emplace_back(accScore, pBestKF);
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        double minScoreToRetain = 0.75f * bestAccScore;

        std::set<KeyFrame *> spAlreadyAddedKF;
        std::vector<KeyFrame *> vpLoopCandidates;
        vpLoopCandidates.reserve(lAccScoreAndMatch.size());

        for (auto &it: lAccScoreAndMatch) {
            if (it.first > minScoreToRetain) {
                KeyFrame *pKFi = it.second;
                if (!spAlreadyAddedKF.count(pKFi)) {
                    vpLoopCandidates.push_back(pKFi);
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        }


        return vpLoopCandidates;
    }

    std::vector<KeyFrame *> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F) {
        std::list<KeyFrame *> lKFsSharingWords;

        // Search all keyframes that share a word with current frame
        {
            std::unique_lock<std::mutex> lock(mMutex);
            for (auto vit = F->mBowVec.begin(), vend = F->mBowVec.end(); vit != vend; vit++) {
                std::list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];
                for (auto pKFi: lKFs) {
                    if (pKFi->mnRelocQuery != F->mnId) {
                        pKFi->mnRelocWords = 0;
                        pKFi->mnRelocQuery = F->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                    pKFi->mnRelocWords++;
                }
            }
        }
        if (lKFsSharingWords.empty())
            return {};

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (auto &lKFsSharingWord: lKFsSharingWords) {
            if (lKFsSharingWord->mnRelocWords > maxCommonWords)
                maxCommonWords = lKFsSharingWord->mnRelocWords;
        }

        int minCommonWords = std::ceil(maxCommonWords * 0.8);

        std::list<std::pair<float, KeyFrame *> > lScoreAndMatch;

        int nscores = 0;

        // Compute similarity score.
        for (auto pKFi: lKFsSharingWords) {
            if (pKFi->mnRelocWords > minCommonWords) {
                nscores++;
                double si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
                pKFi->mRelocScore = si;
                lScoreAndMatch.emplace_back(si, pKFi);
            }
        }

        if (lScoreAndMatch.empty())
            return {};

        std::list<std::pair<float, KeyFrame *> > lAccScoreAndMatch;
        double bestAccScore = 0;

        // Lets now accumulate score by covisibility
        for (auto &it: lScoreAndMatch) {
            KeyFrame *pKFi = it.second;
            std::vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            double bestScore = it.first;
            double accScore = bestScore;
            KeyFrame *pBestKF = pKFi;
            for (auto pKF2: vpNeighs) {
                if (pKF2->mnRelocQuery != F->mnId)
                    continue;

                accScore += pKF2->mRelocScore;
                if (pKF2->mRelocScore > bestScore) {
                    pBestKF = pKF2;
                    bestScore = pKF2->mRelocScore;
                }

            }
            lAccScoreAndMatch.emplace_back(accScore, pBestKF);
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        double minScoreToRetain = 0.75 * bestAccScore;
        std::set<KeyFrame *> spAlreadyAddedKF;
        std::vector<KeyFrame *> vpRelocCandidates;
        vpRelocCandidates.reserve(lAccScoreAndMatch.size());
        for (auto &it: lAccScoreAndMatch) {
            const float &si = it.first;
            if (si > minScoreToRetain) {
                KeyFrame *pKFi = it.second;
                if (!spAlreadyAddedKF.count(pKFi)) {
                    vpRelocCandidates.push_back(pKFi);
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        }

        return vpRelocCandidates;
    }

} //namespace ORB_SLAM
