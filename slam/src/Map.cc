#include "Map.h"

#include<mutex>

#define TEST_DATA 0xdeadbeef

namespace ORB_SLAM2 {

    Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0) {
    }

    template<class Archive>
    void Map::save(Archive &ar, const unsigned int version) const {
        unsigned int test_data = TEST_DATA;
        int nItems = mspMapPoints.size();
        ar & nItems;
        std::cout << "{INFO}mspMapPoints size = " << nItems << std::endl;

        std::for_each(mspMapPoints.begin(), mspMapPoints.end(),
                      [&ar](const std::pair<std::shared_ptr<MapPoint>, unsigned long>& pMapPoint) {
                          ar & *pMapPoint.first;
                      });

        nItems = mspKeyFrames.size();
        std::cout << "{INFO}mspKeyFrames size = " << nItems << std::endl;
        ar & nItems;
        std::for_each(mspKeyFrames.begin(), mspKeyFrames.end(), [&ar](std::pair<KeyFrame *, unsigned long> pKeyFrame) {
            ar & *pKeyFrame.first;
        });

        nItems = mvpKeyFrameOrigins.size();
        std::cout << "{INFO}mvpKeyFrameOrigins size = " << nItems << std::endl;
        ar & nItems;
        std::for_each(mvpKeyFrameOrigins.begin(), mvpKeyFrameOrigins.end(), [&ar](KeyFrame *pKeyFrameOrigin) {
            ar & *pKeyFrameOrigin;
        });
        // Pertaining to map drawing
        //nItems = mvpReferenceMapPoints.size();
        //cout << "$${INFO}mvpReferenceMapPoints size = %d " << nItems << endl;
        //ar & nItems;
        //std::for_each(mvpReferenceMapPoints.begin(), mvpReferenceMapPoints.end(), [&ar](MapPoint* pMapPointReference) {
        //    ar & *pMapPointReference;
        //});
        ar & const_cast<long unsigned int &> (mnMaxKFid);

        ar & test_data;
    }

    template<class Archive>
    void Map::load(Archive &ar, const unsigned int version) {
        unsigned int test_data;

        int nItems;
        ar & nItems;
        std::cout << "{INFO}mspMapPoints size = " << nItems << std::endl;

        for (int i = 0; i < nItems; ++i) {

            std::shared_ptr<MapPoint> pMapPoint = std::make_shared<MapPoint>();
            ar & *pMapPoint;
            mspMapPoints.insert({pMapPoint, i});
        }

        ar & nItems;
        std::cout << "{INFO}mspKeyFrames size = " << nItems << std::endl;

        for (int i = 0; i < nItems; ++i) {

            auto *pKeyFrame = new KeyFrame;
            ar & *pKeyFrame;
            mspKeyFrames.insert({pKeyFrame, i});
        }


        ar & nItems;
        std::cout << "{INFO}mvpKeyFrameOrigins size = " << nItems << std::endl;

        for (int i = 0; i < nItems; ++i) {

            auto *pKeyFrame = new KeyFrame;
            ar & *pKeyFrame;
            mvpKeyFrameOrigins.push_back(mspKeyFrames.begin()->first);
        }

        ar & const_cast<long unsigned int &> (mnMaxKFid);

        ar & test_data;
        if (test_data == TEST_DATA)
            std::cout << ">>Map Loading Validated as True" << std::endl;
        else
            std::cout << "ERROR Map Loading Validated as False: Got -" << test_data << " :( Check Load Save sequence"
                      << std::endl;

    }


// Explicit template instantiation
    template void Map::save<boost::archive::binary_oarchive>(
            boost::archive::binary_oarchive &,
            const unsigned int) const;

    template void Map::save<boost::archive::binary_iarchive>(
            boost::archive::binary_iarchive &,
            const unsigned int) const;

    template void Map::load<boost::archive::binary_oarchive>(
            boost::archive::binary_oarchive &,
            const unsigned int);

    template void Map::load<boost::archive::binary_iarchive>(
            boost::archive::binary_iarchive &,
            const unsigned int);

    void Map::AddKeyFrame(KeyFrame *pKF) {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        if (!mspKeyFrames.count(pKF)) {
            mspKeyFrames.insert({pKF, pKF->mnId});

            if (pKF->mnId > mnMaxKFid)
                mnMaxKFid = pKF->mnId;
        }
    }

    void Map::AddMapPoint(std::shared_ptr<MapPoint> &pMP) {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        //if (!mspMapPoints.count(pMP)) {
        mspMapPoints[pMP] = pMP->mnId;
        //}
    }

    void Map::EraseMapPoint(std::shared_ptr<MapPoint> &pMP) {
        //std::unique_lock<std::mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

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
            mapPoints.emplace_back(mapPoint.first);
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
        for (auto &mspMapPoint: mspMapPoints) {
            //delete mspMapPoint.first;
            //mspMapPoint.first = nullptr;
            auto point = mspMapPoint.first;
            point = nullptr;
        }

        for (auto mspKeyFrame: mspKeyFrames) {
            delete mspKeyFrame.first;
            //auto keyFrame = mspKeyFrame.first;
            //keyFrame = nullptr;
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
        mspMapPoints.clear();
        mvpReferenceMapPoints.clear();
        mvpReferenceMapPoints.clear();
    }

} //namespace ORB_SLAM
