/*************************************************************************
	> File Name: cost.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 18 Apr 2017 01:43:55 PM PDT
 ************************************************************************/

#ifndef _COST_H
#define _COST_H

#include <opencv2/opencv.hpp>
#include <armadillo>
#include <cmath>
#include <boost/filesystem.hpp>
#include <config.h>

namespace markov_segm
{
    class Cost 
    {
        public:
            Cost();
            void init();
            double c2_test(cv::Mat& prob, int i, int j, int classe);
            double c2_potts(cv::Mat& prob, int i, int j, int classe);
            double c1(cv::Mat& img, int i, int j, int classe);
            double compute(cv::Mat& img, int i, int j, int classe, cv::Mat& prob);

        private:
            void fix_singular_(int classe); // fix the singular matrix 
            void compute_mean_variance_(cv::Mat& image, int classe);
            void compute_covariance_(cv::Mat& image, int classe);

            arma::vec3 mean_[NB_COLORS];
            arma::mat33 covariance_[NB_COLORS];
            arma::mat33 inv_covariance_[NB_COLORS];
    };
}
#endif
