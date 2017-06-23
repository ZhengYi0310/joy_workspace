/*************************************************************************
	> File Name: texture.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 21 Apr 2017 10:39:33 AM PDT
 ************************************************************************/

#ifndef _TEXTURE_H
#define _TEXTURE_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

#include <armadillo>

namespace markov_segm
{
    class Texture 
    {
        public:
            Texture() {};
            virtual ~Texture() {};
            void init();
            void computeTexture(cv::Mat& img);
            void applyGaborFilter(cv::Mat& img);
            void nonlinearMapping(std::vector<cv::Mat>& img_vec);
            void applyGaussianBlur(std::vector<cv::Mat>& img_vec);
            std::vector<double> getWavelenVec();
            std::vector<cv::Mat> getGaborFilteredImgs();

        private:
            std::vector<double> mu_0_;
            std::vector<cv::Mat> gabor_filtered_imgs;
            std::vector<double> thetas;
    };
}
#endif
