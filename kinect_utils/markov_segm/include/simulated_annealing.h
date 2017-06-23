/*************************************************************************
	> File Name: simulated_annealing.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 18 Apr 2017 03:33:51 PM PDT
 ************************************************************************/

#ifndef _SIMULATED_ANNEALING_H
#define _SIMULATED_ANNEALING_H

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <random>

#include <opencv2/opencv.hpp>

//#include <random.h>
#include <convert.h>
#include <cost.h>
#include <texture.h>

#include <markov_segm/Run.h>
#include <markov_segm/Init.h>

// default capture width and height with kinect
const int FRAME_HEIGHT = 480;
const int FRAME_WIDTH = 640;
namespace markov_segm
{
    class SimulatedAnnealing
    {
        public:
            SimulatedAnnealing(ros::NodeHandle& node_handle);
            virtual ~SimulatedAnnealing() {};

            bool runCallback(markov_segm::Run::Request &req,
                             markov_segm::Run::Response &res);
            bool initCallback (markov_segm::Init::Request &req,
                               markov_segm::Init::Response &res);

        private:
            void init(cv::Mat &img);
            void run();
            void random_image(cv::Mat& img);
            void prepareTexture();

            // dynamic array used to store the number of gabor filter angles and the central frequency;
            //double *mu_0;
            std::vector<double> mu_0;
            static double theta[4];
            std::vector<cv::Mat> gabor_filtered_imgs;
            std::vector<cv::Mat> feature_img;

            std::string image_path_;
            std::string image_name_;

            static std::mt19937 eng_;
            std::uniform_int_distribution<uint32_t> uint_dist_;
            std::uniform_real_distribution<double> ureal_dist_;
            
            ros::NodeHandle node_handle_;

            cv::Mat prob_;
            cv::Mat output_;
            cv::Mat img_;
            cv::Mat img_gray_;

            double delta_global_energy_;
            double delta_;
            double temperature_;
            int new_class_;
            Cost c_;

            //ros::ServiceServer initialize_server_;
            ros::ServiceServer run_server_;
            ros::ServiceServer init_server_;            
    };

    //std::mt19937 SimulatedAnnealing::eng_(time(NULL));
}
#endif
