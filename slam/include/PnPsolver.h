
#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2 {

    class PnPsolver {
    public:
        PnPsolver(const Frame &F, const std::vector<MapPoint *> &vpMapPointMatches);

        ~PnPsolver();

        void SetRansacParameters(double probability = 0.99, int minInliers = 8, int maxIterations = 300, int minSet = 4,
                                 float epsilon = 0.4,
                                 float th2 = 5.991);

        cv::Mat find(std::vector<bool> &vbInliers, int &nInliers);

        cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

    private:

        void CheckInliers();

        bool Refine();

        // Functions from the original EPnP code
        void set_maximum_number_of_correspondences(int n);

        void reset_correspondences();

        void add_correspondence(double X, double Y, double Z,
                                double u, double v);

        double compute_pose(double R[3][3], double T[3]);

        double reprojection_error(const double R[3][3], const double t[3]);

        void choose_control_points();

        void compute_barycentric_coordinates();

        void fill_M(CvMat *M, int row, const double *alphas, double u, double v) const;

        void compute_ccs(const double *betas, const double *ut);

        void compute_pcs();

        void solve_for_sign();

        static void find_betas_approx_1(const CvMat *L_6x10, const CvMat *Rho, double *betas);

        static void find_betas_approx_2(const CvMat *L_6x10, const CvMat *Rho, double *betas);

        static void find_betas_approx_3(const CvMat *L_6x10, const CvMat *Rho, double *betas);

        static void qr_solve(CvMat *A, CvMat *b, CvMat *X);

        static double dot(const double *v1, const double *v2);

        static double dist2(const double *p1, const double *p2);

        void compute_rho(double *rho);

        static void compute_L_6x10(const double *ut, double *l_6x10);

        static void gauss_newton(const CvMat *L_6x10, const CvMat *Rho, double current_betas[4]);

        static void compute_A_and_b_gauss_newton(const double *l_6x10, const double *rho,
                                                 const double cb[4], CvMat *A, CvMat *b);

        double compute_R_and_t(const double *ut, const double *betas,
                               double R[3][3], double t[3]);

        void estimate_R_and_t(double R[3][3], double t[3]);

        static void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
                                 double R_src[3][3], double t_src[3]);

        static void mat_to_quat(const double R[3][3], double q[4]);


        double uc, vc, fu, fv;

        double *pws, *us, *alphas, *pcs;
        int maximum_number_of_correspondences;
        int number_of_correspondences;

        double cws[4][3], ccs[4][3];

        std::vector<MapPoint *> mvpMapPointMatches;

        // 2D Points
        std::vector<cv::Point2f> mvP2D;
        std::vector<float> mvSigma2;

        // 3D Points
        std::vector<cv::Point3f> mvP3Dw;

        // Index in Frame
        std::vector<size_t> mvKeyPointIndices;

        // Current Estimation
        double mRi[3][3];
        double mti[3];
        std::vector<bool> mvbInliersi;
        int mnInliersi;

        // Current Ransac State
        int mnIterations;
        std::vector<bool> mvbBestInliers;
        int mnBestInliers;
        cv::Mat mBestTcw;

        // Refined
        cv::Mat mRefinedTcw;
        std::vector<bool> mvbRefinedInliers;
        int mnRefinedInliers;

        // Number of Correspondences
        int N;

        // Indices for random selection [0 .. N-1]
        std::vector<size_t> mvAllIndices;

        // RANSAC probability
        double mRansacProb;

        // RANSAC min inliers
        int mRansacMinInliers;

        // RANSAC max iterations
        int mRansacMaxIts;

        // RANSAC expected inliers/total ratio
        float mRansacEpsilon;


        // RANSAC Minimun Set used at each iteration
        int mRansacMinSet;

        // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
        std::vector<float> mvMaxError;

    };

} //namespace ORB_SLAM

#endif //PNPSOLVER_H
