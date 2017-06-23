/*************************************************************************
	> File Name: texture.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 21 Apr 2017 10:58:43 AM PDT
 ************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <cassert>

#include <texture.h>
namespace markov_segm
{
    void Texture::init()
    {
        //heuristics of choosing the angles of gabor filters 
        for (int i = 3; i <= 7; i++)
        {
            thetas.push_back(double(i) / 8 * CV_PI);
        }
    }

    void Texture::computeTexture(cv::Mat& img)
    {
        int max_power = log(img.cols / 8) / log(2); //heuristics of choosing the number of gabor filters 
        for (int i = 4; i <= max_power; i++)
        {
            mu_0_.push_back(pow(1.4486, i) / CV_PI); // heuristics of choosing the sinusoidal wavelength 
        }

        // convert the image to float 
        cv::Mat src_temp;
        img.convertTo(src_temp, CV_64F, 1/255.0);

        // apply the gabor filer back to it 
        applyGaborFilter(src_temp);
        //applyGaussianBlur(gabor_filtered_imgs);
    }

    void Texture::applyGaborFilter(cv::Mat& img)
    {
        // heuristics of choosing other gabor filter params 
        int kernel_size = 21;
        double lm = 1.0, gm = 0.5, ps = 0;
        double sig;

        for (int i = 0; i < thetas.size(); i++)
        {
            for (int j = 0; j < mu_0_.size(); j++)
            {
                sig = 5.0930 / mu_0_[j];
                cv::Mat kernel = cv::getGaborKernel(cv::Size(kernel_size, kernel_size), sig, thetas[i], mu_0_[j], gm, ps, CV_64F);
                //std::cout << thetas[i] << " " << mu_0_[j] << " " << index << std::endl;
                cv::Mat dest;
                cv::filter2D(img, dest, -1, kernel, cv::Point(-1, -1));
                gabor_filtered_imgs.push_back(dest);

                //gabor_filtered_imgs[index].convertTo(gabor_filtered_imgs[index], CV_32F, 1.0 / 256.0); // move the image to proper range in order to show it.
            }
        }

        nonlinearMapping(gabor_filtered_imgs);
    }

    void Texture::nonlinearMapping(std::vector<cv::Mat>& img_vec)
    {
        // use Armadillo library to to nonlinear mapping 
        for (int i = 0; i < img_vec.size(); i++)
        {
             img_vec[i].convertTo(img_vec[i], CV_64F);
            arma::mat img_arma(reinterpret_cast<double*>(img_vec[i].data), img_vec[i].rows, img_vec[i].cols);
            //std::cout << img_vec[i].rows << " " << img_vec[i].cols << " " << size(img_arma);
            // conduct the nonlinear mapping on the amra matrix 
            //img_arma = abs(tanh(img_arma)); // 1 is set via heuristics
            img_arma = tanh(img_arma * 0.25);
            //std::cout << max(vectorise(img_arma)) << " " << min(vectorise(img_arma)) << std::endl;
            img_arma = (img_arma - min(vectorise(img_arma))); 
            img_arma = img_arma / max(vectorise(img_arma));
            std::cout << max(vectorise(img_arma)) << " " << min(vectorise(img_arma)) << std::endl;
            img_arma = img_arma * 255; // normalize image arma to [0, 255]
            // convert the arma mat back to opencv Mat
            std::cout << max(vectorise(img_arma)) << " " << min(vectorise(img_arma)) << std::endl;            
            cv::Mat temp(img_vec[i].rows, img_vec[i].cols, CV_64F, img_arma.memptr()); // has to be CV_64FC1, cauest that's the accuracy for double type
            temp.copyTo(img_vec[i]);
            img_vec[i].convertTo(img_vec[i], CV_8U);
            //std::cerr << img_vec[i](cv::Rect(300,30,200,30)) << std::endl;
            
        }
    }

    void Texture::applyGaussianBlur(std::vector<cv::Mat>& img_vec)
    {
        // Blur the grayscale image with gaussian distribution
        for (int i = 0; i < img_vec.size(); i++)
        {
            cv::GaussianBlur(img_vec[i], img_vec[i], cv::Size(5, 5), mu_0_[i % mu_0_.size()], mu_0_[i % mu_0_.size()]);
            //std::cout << mu_0_[i % mu_0_.size()] << std::endl;
        }
    }

    std::vector<double> Texture::getWavelenVec()
    {
        return mu_0_;
    }

    std::vector<cv::Mat> Texture::getGaborFilteredImgs()
    {
        return gabor_filtered_imgs;
    }
}

