

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "../Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "../Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

//#include <mutex>


namespace ORB_SLAM2 {
    struct id_map {
        bool is_valid;
        long unsigned int id;
    };

    class Map;

    class MapPoint;

    class Frame;

    class KeyFrameDatabase;

    class KeyFrame {
    public:
        KeyFrame();

        ~KeyFrame();

        KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

        // Pose functions
        void SetPose(const cv::Mat &Tcw);

        cv::Mat GetPose();

        cv::Mat GetPoseInverse();

        cv::Mat GetCameraCenter();

        cv::Mat GetRotation();

        cv::Mat GetTranslation();

        // Bag of Words Representation
        void ComputeBoW();

        // Covisibility graph functions
        void AddConnection(KeyFrame *pKF, const int &weight);

        void EraseConnection(KeyFrame *pKF);

        void UpdateConnections();

        void UpdateBestCovisibles();

        std::set<KeyFrame *> GetConnectedKeyFrames();

        std::unordered_map<std::shared_ptr<MapPoint>, int> GetMapPointsDic();

        std::unordered_map<KeyFrame *, int> GetConnectedKeyFramesAsDic();

        std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();

        std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);

        std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);

        int GetWeight(KeyFrame *pKF);

        // Spanning tree functions
        void AddChild(KeyFrame *pKF);

        void EraseChild(KeyFrame *pKF);

        void ChangeParent(KeyFrame *pKF);

        std::set<KeyFrame *> GetChilds();

        KeyFrame *GetParent();

        bool hasChild(KeyFrame *pKF);

        // Loop Edges
        void AddLoopEdge(KeyFrame *pKF);

        std::set<KeyFrame *> GetLoopEdges();

        // MapPoint observation functions
        void AddMapPoint(std::shared_ptr<MapPoint> pMP, const size_t &idx);

        void EraseMapPointMatch(const size_t &idx);

        void EraseMapPointMatch(std::shared_ptr<MapPoint> &pMP);

        void ReplaceMapPointMatch(const size_t &idx, std::shared_ptr<MapPoint> pMP);

        std::set<std::shared_ptr<MapPoint>> GetMapPoints();

        std::unordered_map<size_t, std::shared_ptr<MapPoint>> GetMapPointMatches();

        int TrackedMapPoints(const int &minObs);

        std::shared_ptr<MapPoint> GetMapPoint(const size_t &idx);

        // KeyPoint functions
        std::vector<size_t> GetFeaturesInArea(const double &x, const double &y, const double &r) const;

        [[maybe_unused]] cv::Mat UnprojectStereo(int i);

        // Image
        bool IsInImage(const double &x, const double &y) const;

        // Enable/Disable bad flag changes
        void SetNotErase();

        void SetErase();

        // Set/check bad flag
        void SetBadFlag();

        bool isBad();

        // Compute Scene Depth (q=2 median). Used in monocular.
        float ComputeSceneMedianDepth(const int q);

        static bool weightComp(int a, int b) {
            return a > b;
        }

        static bool lId(KeyFrame *pKF1, KeyFrame *pKF2) {
            return pKF1->mnId < pKF2->mnId;
        }

        void SetMap(Map *map);

        void SetKeyFrameDatabase(const std::shared_ptr<KeyFrameDatabase> &pKeyFrameDB);

        void SetORBvocabulary(std::shared_ptr<ORBVocabulary> pORBvocabulary);

        void SetMapPoints(std::vector<std::shared_ptr<MapPoint>> spMapPoints);

        void SetSpanningTree(std::vector<KeyFrame *> vpKeyFrames);

        void SetGridParams(std::vector<KeyFrame *> vpKeyFrames);

        // The following variables are accesed from only 1 thread or never change (no mutex needed).
    public:

        static long unsigned int nNextId;
        long unsigned int mnId;
        const long unsigned int mnFrameId;

        const double mTimeStamp;
        cv::Mat mTcp;
        // Grid (to speed up feature matching)
        const int mnGridCols;
        const int mnGridRows;
        const float mfGridElementWidthInv;
        const float mfGridElementHeightInv;

        // Variables used by the tracking
        long unsigned int mnTrackReferenceForFrame;
        long unsigned int mnFuseTargetForKF;

        // Variables used by the local mapping
        long unsigned int mnBALocalForKF;
        long unsigned int mnBAFixedForKF;

        // Variables used by the keyframe database
        long unsigned int mnLoopQuery;
        int mnLoopWords;
        double mLoopScore;
        long unsigned int mnRelocQuery;
        int mnRelocWords;
        double mRelocScore;

        // Variables used by loop closing
        cv::Mat mTcwGBA;
        cv::Mat mTcwBefGBA;
        long unsigned int mnBAGlobalForKF;

        // Calibration parameters
        const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

        // Number of KeyPoints
        const int N;

        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        const std::vector<cv::KeyPoint> mvKeys;
        const std::vector<cv::KeyPoint> mvKeysUn;
        const std::vector<float> mvuRight; // negative value for monocular points
        const std::vector<float> mvDepth; // negative value for monocular points
        const cv::Mat mDescriptors;

        //BoW
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        std::map<long unsigned int, int> mConnectedKeyFrameWeights_nId;
        std::map<long unsigned int, id_map> mvpOrderedConnectedKeyFrames_nId;
        id_map mparent_KfId_map;
        std::map<long unsigned int, id_map> mmChildrens_nId;
        std::map<long unsigned int, id_map> mmLoopEdges_nId;
        // Scale
        const int mnScaleLevels;
        const float mfScaleFactor;
        const float mfLogScaleFactor;
        const std::vector<float> mvScaleFactors;
        const std::vector<float> mvLevelSigma2;
        const std::vector<float> mvInvLevelSigma2;

        // Image bounds and calibration
        const int mnMinX;
        const int mnMinY;
        const int mnMaxX;
        const int mnMaxY;
        const cv::Mat mK;


        // The following variables need to be accessed trough a mutex to be thread safe.
    protected:

        // SE3 Pose and camera center
        cv::Mat Tcw;
        cv::Mat Twc;
        cv::Mat Ow;
        std::map<long unsigned int, id_map> mmMapPoints_nId;

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            boost::serialization::split_member(ar, *this, version);
        }

        template<class Archive>
        void save(Archive &ar, const unsigned int version) const;


        template<class Archive>
        void load(Archive &ar, const unsigned int version);

        // MapPoints associated to keypoints
        std::unordered_map<size_t, std::shared_ptr<MapPoint>> mvpMapPoints;

        // BoW
        std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
        std::shared_ptr<ORBVocabulary> mpORBvocabulary;

        // Grid over the image to speed up feature matching
        std::vector<std::vector<std::vector<size_t> > > mGrid;

        std::unordered_map<KeyFrame *, int> mConnectedKeyFrameWeights;
        std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames;
        std::vector<int> mvOrderedWeights;

        // Spanning Tree and Loop Edges
        bool mbFirstConnection;
        KeyFrame *mpParent;
        std::set<KeyFrame *> mspChildrens;
        std::set<KeyFrame *> mspLoopEdges;

        // Bad flags
        bool mbNotErase;
        bool mbToBeErased;
        bool mbBad;

        float mHalfBaseline; // Only for visualization

        Map *mpMap;

        std::mutex mMutexPose;
        std::mutex mMutexConnections;
        std::mutex mMutexFeatures;
    };

}

#endif // KEYFRAME_H
