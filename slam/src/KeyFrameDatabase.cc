

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"

//#include<mutex>


namespace ORB_SLAM2 {

    KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary &voc) :
            mpVoc(&voc) {
        //mvInvertedFile.resize(voc.size());
    }


    template<class Archive>
    void KeyFrameDatabase::save(Archive & ar, const unsigned int version) const
    {
        int nItems_a, nItems_b;
        nItems_a = mvInvertedFile.size();
        ar & nItems_a;
        std::cout << "{INFO}Database elmnts = %d " << nItems_a << std::endl;

        for (const auto &[index,it] : mvInvertedFile) {
            nItems_b = it.size();
            std::cout << "{INFO}kfs no elmnts = %d " << nItems_b <<std:: endl;

            ar & nItems_b;
            for (auto lit : it) {
                ar & lit;
            }
        }
#if 0
        std::for_each(mvInvertedFile.begin(), mvInvertedFile.end(), [&ar](list<KeyFrame*>* plKeyFrame) {

            nItems_b = (*plKeyFrame).size();
            ar & nItems_b;
            std::for_each((*plKeyFrame).begin(), (*plKeyFrame).end(), [&ar](KeyFrame* pKeyFrame) {
                ar & *pKeyFrame;
            });



        });
#endif

    }

    template<class Archive>
    void KeyFrameDatabase::load(Archive & ar, const unsigned int version)
    {
        int nItems_a, nItems_b;
        int j , i;
        std::list<KeyFrame*> temp_list;
        ar & nItems_a;
        std::cout << "{INFO}Database elmnts = %d " << nItems_a << std::endl;

        for (i = 0; i < nItems_a; ++i) {

            ar & nItems_b;
            std::cout << "{INFO}kfs no elmnts = %d " << nItems_b << std::endl;

            for (j = 0; j < nItems_b; ++j) {
                KeyFrame* pKeyFrame = new KeyFrame;
                ar & *pKeyFrame;
                mvInvertedFile.at(i).at(pKeyFrame) = j;
            }
        }
    }


// Explicit template instantiation
    template void KeyFrameDatabase::save<boost::archive::binary_oarchive>(
            boost::archive::binary_oarchive &,
            const unsigned int) const;
    template void KeyFrameDatabase::save<boost::archive::binary_iarchive>(
            boost::archive::binary_iarchive &,
            const unsigned int) const;
    template void KeyFrameDatabase::load<boost::archive::binary_oarchive>(
            boost::archive::binary_oarchive &,
            const unsigned int);
    template void KeyFrameDatabase::load<boost::archive::binary_iarchive>(
            boost::archive::binary_iarchive &,
            const unsigned int);


    void KeyFrameDatabase::add(KeyFrame *pKF) {
        for (const auto &vit: pKF->mBowVec) {
            mvInvertedFile[vit.first][pKF] = 1;
        }
    }

    void KeyFrameDatabase::erase(KeyFrame *pKF) {
        //std::unique_lock<std::mutex> lock(mMutex);
        for (const auto &vit: pKF->mBowVec) {
            mvInvertedFile[vit.first].erase(pKF);
            /*std::list<KeyFrame *> &lKFs = mvInvertedFile[vit.first];

            for (auto lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
                if (pKF == *lit) {
                    lKFs.erase(lit);
                    break;
                }
            }*/
        }
    }

    void KeyFrameDatabase::clear() {
        mvInvertedFile.clear();
        //mvInvertedFile.resize(mpVoc->size());
    }


    std::vector<KeyFrame *> KeyFrameDatabase::DetectLoopCandidates(KeyFrame *pKF, double minScore) {
        std::set<KeyFrame *> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
        std::list<KeyFrame *> lKFsSharingWords;

        for (const auto &vit: pKF->mBowVec) {
            for (auto[pKFi, junk]: mvInvertedFile[vit.first]) {
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

        //std::unique_lock<std::mutex> lock(mMutex);
        for (const auto &vit: F->mBowVec) {
            if (mvInvertedFile.count(vit.first)) {
                for (auto[pKFi, junk]: mvInvertedFile[vit.first]) {
                    if (pKFi->mnRelocQuery != F->mnId) {
                        pKFi->mnRelocWords = 0;
                        pKFi->mnRelocQuery = F->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                    pKFi->mnRelocWords++;
                }
            }
        }

        if (lKFsSharingWords.empty()) {
            return {};

        }
        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (auto &lKFsSharingWord: lKFsSharingWords) {
            if (lKFsSharingWord->mnRelocWords > maxCommonWords)
                maxCommonWords = lKFsSharingWord->mnRelocWords;
        }
        int minCommonWords = std::ceil(maxCommonWords * 0.5);
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

        if (lScoreAndMatch.empty()) {
            return {};

        }

        std::list<std::pair<float, KeyFrame *> > lAccScoreAndMatch;
        double bestAccScore = 0;
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
