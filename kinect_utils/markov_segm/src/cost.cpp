/*************************************************************************
	> File Name: cost.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 18 Apr 2017 01:56:02 PM PDT
 ************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <cassert>

#include <ros/ros.h>
#include <ros/package.h>

#include <cost.h>
using namespace std;

namespace markov_segm
{
    #ifndef M_PI
    #define M_PI        3.14159265358979323846
    #endif

    Cost::Cost()
    {
        for (int i = 0; i < NB_COLORS; i++)
        {
            covariance_[i].zeros();
            inv_covariance_[i].zeros();
        }
    }

    void Cost::compute_mean_variance_(cv::Mat& image, int classe)
    {
        double sum, sum_sqr;

        for (int k = 0; k <= 2; k++)
        {
            sum = 0;
            sum_sqr = 0;

            for (int i = 0; i < image.rows; i++)
            {
                for (int j = 0; j < image.cols; j++)
                {
                    sum += image.at<cv::Vec3b>(i, j)[k];
                    sum_sqr += image.at<cv::Vec3b>(i ,j)[k] * image.at<cv::Vec3b>(i, j)[k];
                }
            }

            mean_[classe](k) = sum / (image.rows * image.cols);
            covariance_[classe](k, k) = (sum_sqr - sum * sum / (image.rows * image.cols)) / (image.rows * image.cols);
        }
    }

    void Cost::compute_covariance_(cv::Mat& image, int classe)
    {
        double sum[3] = {0, 0, 0};

        for (int i = 0; i < image.rows; i++)
        {
            for (int j = 0; j < image.cols; j++)
            {
                sum[0] += (image.at<cv::Vec3b>(i, j)[0] - mean_[classe](0)) * (image.at<cv::Vec3b>(i, j)[1] - mean_[classe](1));
                sum[1] += (image.at<cv::Vec3b>(i, j)[0] - mean_[classe](0)) * (image.at<cv::Vec3b>(i, j)[2] - mean_[classe](2));
                sum[2] += (image.at<cv::Vec3b>(i, j)[1] - mean_[classe](1)) * (image.at<cv::Vec3b>(i, j)[2] - mean_[classe](2));
            }
        }

        covariance_[classe](0, 1) = sum[0] / (image.rows * image.cols);
        covariance_[classe](0, 2) = sum[1] / (image.rows * image.cols);
        covariance_[classe](1, 2) = sum[2] / (image.rows * image.cols);

        covariance_[classe](1, 0) = covariance_[classe](0 ,1);
        covariance_[classe](2, 0) = covariance_[classe](0, 2);
        covariance_[classe](2, 1) = covariance_[classe](1, 2);
    }

    void Cost::fix_singular_(int classe)
    {
        // Avoid singular matrix
        for (arma::uword i = 0; i < covariance_[classe].n_rows; i++)
        {
            for (arma::uword j = 0; j < covariance_[classe].n_cols; j++)
            {
                if (covariance_[classe](i, j) == 0)
                {
                    covariance_[classe](i, j) = 1e-10;
                }
            }
        }
    }

    void Cost::init()
    {
        cv::Mat image;
        boost::filesystem::directory_iterator end;

        std::string input_dir(std::string(ros::package::getPath("markov_segm")) + "/classe");
        int classe = 0;

        if (boost::filesystem::exists(input_dir) && boost::filesystem::is_directory(input_dir))
        {
            for (boost::filesystem::directory_iterator it(input_dir); it != end; it++)
            {
                if (it->path().extension() == ".png")
                {
                    std::cout << it->path().string() << " corresponding to class " << classe << std::endl;
                    image = cv::imread(it->path().string(), CV_LOAD_IMAGE_COLOR);
                    cv::cvtColor(image, image, CV_BGR2Luv);

                    compute_mean_variance_(image, classe);
                    compute_covariance_(image, classe);

                    fix_singular_(classe);
                    inv_covariance_[classe] = arma::inv(covariance_[classe]);

                    classe++;
                }
            }
        }
    }

    double Cost::c2_test(cv::Mat& prob, int i, int j, int classe)
    {
        if (prob.at<uchar>(i, j) == classe)
        {
            return -Config::beta;
        }
        else
        {
            return Config::beta;
        }
    }

    double Cost::c2_potts(cv::Mat& prob, int i, int j, int classe)
    {
        double cost = 0;

        if (i > 0)
        {
            cost += c2_test(prob, i - 1, j, classe);
        }

        if (i < (prob.rows - 1))
        {
            cost += c2_test(prob, i + 1, j, classe);
        }

        if (j > 0)
        {
            cost += c2_test(prob, i, j - 1, classe);
        }

        if (j < (prob.cols - 1))
        {
            cost += c2_test(prob, i, j + 1, classe);
        }

        return cost;

    }

    double Cost::c1(cv::Mat& img, int i, int j, int classe)
    {
        arma::vec3 x;
        x(0) = img.at<cv::Vec3b>(i, j)[0];
        x(1) = img.at<cv::Vec3b>(i, j)[1];
        x(2) = img.at<cv::Vec3b>(i, j)[2];

        arma::vec3 m = (x - mean_[classe]);
        arma::mat out = trans(m) * inv_covariance_[classe] * m;

        return log(sqrt(2.0 * M_PI * arma::det(covariance_[classe]))) + 0.5 * out[0];
    }

    double Cost::compute(cv::Mat& img, int i, int j, int classe, cv::Mat& prob)
    {
        return c1(img, i, j, classe) + c2_potts(prob, i, j, classe);
    }
}

