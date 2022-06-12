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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"


#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/unordered_map.hpp>
#include<opencv2/core/core.hpp>
#include<mutex>

#include "ORBmatcher.h"

namespace ORB_SLAM2 {

    class KeyFrame;

    class Map;

    class Frame;


    class MapPoint {
    public:
        MapPoint();            /* Default constructor for serialization */

        MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap);

        MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF);

        void SetWorldPos(const cv::Mat &Pos);

        cv::Mat GetWorldPos();

        cv::Mat GetNormal();

        KeyFrame *GetReferenceKeyFrame();

        std::unordered_map<KeyFrame *, size_t> GetObservations();

        int Observations();

        void AddObservation(KeyFrame *pKF, size_t idx);

        void EraseObservation(KeyFrame *pKF);

        int GetIndexInKeyFrame(KeyFrame *pKF);

        bool IsInKeyFrame(KeyFrame *pKF);

        void SetBadFlag();

        bool isBad();

        void Replace(const std::shared_ptr<MapPoint>& pMP);

        std::shared_ptr<MapPoint> GetReplaced();

        void IncreaseVisible(int n = 1);

        void IncreaseFound(int n = 1);

        float GetFoundRatio();

        void ComputeDistinctiveDescriptors();

        cv::Mat GetDescriptor();

        void UpdateNormalAndDepth();

        float GetMinDistanceInvariance();

        float GetMaxDistanceInvariance();

        int PredictScale(const double &currentDist, KeyFrame *pKF);

        int PredictScale(const double &currentDist, Frame *pF);

        void SetMap(Map *map);

        void SetObservations(std::vector<KeyFrame *> spKeyFrames);

    public:
        long unsigned int mnId;
        static long unsigned int nNextId;
        long int mnFirstKFid;
        int nObs;

        // Variables used by the tracking
        float mTrackProjX;
        float mTrackProjY;
        float mTrackProjXR;
        bool mbTrackInView;
        int mnTrackScaleLevel;
        float mTrackViewCos;
        long unsigned int mnTrackReferenceForFrame;
        long unsigned int mnLastFrameSeen;

        // Variables used by local mapping
        long unsigned int mnBALocalForKF;
        long unsigned int mnFuseCandidateForKF;

        // Variables used by loop closing
        long unsigned int mnLoopPointForKF;
        long unsigned int mnCorrectedByKF;
        long unsigned int mnCorrectedReference;
        cv::Mat mPosGBA;
        long unsigned int mnBAGlobalForKF;


        static std::mutex mGlobalMutex;

    protected:

        // Position in absolute coordinates
        cv::Mat mWorldPos;

        // Keyframes observing the point and associated index in keyframe
        std::unordered_map<KeyFrame *, size_t> mObservations;

        // Mean viewing direction
        cv::Mat mNormalVector;

        // Best descriptor to fast matching
        cv::Mat mDescriptor;

        // Reference KeyFrame
        KeyFrame *mpRefKF;

        // Tracking counters
        int mnVisible;
        int mnFound;
        std::map<long unsigned int, size_t> mObservations_nId;


        // Bad flag (we do not currently erase MapPoint from memory)
        bool mbBad;
        std::shared_ptr<MapPoint> mpReplaced;

        // Scale invariance distances
        double mfMinDistance;
        double mfMaxDistance;

        Map *mpMap;

        std::mutex mMutexPos;
        std::mutex mMutexFeatures;
        std::pair<long unsigned int, bool> mref_KfId_pair;

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            boost::serialization::split_member(ar, *this, version);
        }


        template<class Archive>
        void save(Archive &ar, const unsigned int version) const;


        template<class Archive>
        void load(Archive &ar, const unsigned int version);

    };

}

#if 1

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)

namespace boost {
    namespace serialization {

/*** CV KeyFrame ***/
        template<class Archive>
        void serialize(Archive &ar, cv::KeyPoint &kf, const unsigned int version) {
            ar & kf.angle;
            ar & kf.class_id;
            ar & kf.octave;
            ar & kf.response;
            ar & kf.response;
            ar & kf.pt.x;
            ar & kf.pt.y;

        }



/*** Mat ***/
/** Serialization support for cv::Mat */
        template<class Archive>
        void save(Archive &ar, const ::cv::Mat &mat, const unsigned int version) {
            cv::Mat cont_mat = mat.clone(); // Clone matrix so it will be continuous
            size_t elem_size = cont_mat.elemSize();
            size_t elem_type = cont_mat.type();
            int cols = cont_mat.cols;
            int rows = cont_mat.rows;

            ar & cols;
            ar & rows;
            ar & elem_size;
            ar & elem_type;

            size_t data_size = cols * rows * elem_size;
            boost::serialization::binary_object mat_data(mat.data, data_size);
            ar & mat_data;
            // for(size_t dc=0; dc<data_size; dc++){
            //   ar & cont_mat.data[dc];
            // }
            //ar & boost::serialization::make_array(m.ptr(), data_size);
        }

/** Serialization support for cv::Mat */
        template<class Archive>
        void load(Archive &ar, ::cv::Mat &mat, const unsigned int version) {
            int cols, rows;
            size_t elem_size, elem_type;

            ar & cols;
            ar & rows;
            ar & elem_size;
            ar & elem_type;

            mat.create(rows, cols, elem_type);
            size_t data_size = mat.cols * mat.rows * elem_size;

            boost::serialization::binary_object mat_data(mat.data, data_size);
            ar & mat_data;
            // for(size_t dc=0; dc<data_size; dc++){
            //   ar & mat.data[dc];
            // }

            //ar & boost::serialization::make_array(mat.ptr(), data_size);
        }


    }
}
#endif
#endif // MAPPOINT_H
