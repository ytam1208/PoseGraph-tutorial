#ifndef DOCK_ICP_HPP
#define DOCK_ICP_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core/types_c.h>
#include "kdtree.hpp"

// //https://github.com/abreheret/icp-opencv


class dock_ICP
{
public:
	float icp(const cv::Point2f* new_points, int nb_point_new, const cv::Point2f* ref_points, int nb_point_ref, CvMat * r_final, CvMat *t_final, CvTermCriteria criteria);
	float inlierratio = 0.0;
	bool getCovICP(const cv::Point2f* r, int nb_point_ref, const cv::Point2f* m, int nb_ref, float *dxx, float *dxy, float *dyx, float *dyy);

	void getRTMatrixSVD(const cv::Point2f* a, const cv::Point2f* b, int count, CvMat *r, CvMat *t);
	bool getRTMatrixSVD_filter(const cv::Point2f* r, const cv::Point2f* m, int nb_point_ref, int count, CvMat * r_final, CvMat *t_final, float *inlierratio);
	float dist_sq(float *a1, float*a2, int dims);
	cv::Mat drawDistribution(cv::Mat *input, float nomalize = 0.0f);
	
	std::vector<cv::Point2f> RT_ref_data;
	CvMat test_r1;
	CvMat test_t1;
	
	dock_ICP(){}
	~dock_ICP(){}
};


#endif
