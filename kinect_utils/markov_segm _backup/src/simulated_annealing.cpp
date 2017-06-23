/*************************************************************************
	> File Name: simulated_annealing.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 18 Apr 2017 03:53:53 PM PDT
 ************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <cassert>

#include <simulated_annealing.h>
//#include "random.h"

namespace markov_segm
{   // initialize static class members
    std::mt19937 SimulatedAnnealing::eng_(time(NULL));
    double SimulatedAnnealing::theta[4] = {0, 45, 90, 135}; // only choose 4 gabor filter orientations

    
    SimulatedAnnealing::SimulatedAnnealing(ros::NodeHandle& node_handle) : node_handle_(node_handle)
    {
        ROS_INFO("set up the simulated_anneling class, register necessary service servers.");
        
        // register necessary service servers 
        run_server_ = node_handle_.advertiseService("run_simulated_annealing", &SimulatedAnnealing::runCallback, this);
        init_server_ = node_handle_.advertiseService("initialize_simulated_annealing", &SimulatedAnnealing::initCallback, this);
    }
    
    void SimulatedAnnealing::random_image(cv::Mat& img)
    {
        std::mt19937 eng(time(NULL));
        std::uniform_int_distribution<uint32_t> uint_dist;

        // randomly initialize the class 
        for (int i = 0; i < img.rows; ++i)
        {
            for (int j = 0; j < img.cols; j++)
            {
                img.at<uchar>(i, j) = uint_dist(eng) % NB_COLORS;
            }
        }
    }

    void SimulatedAnnealing::prepareTexture()
    {
        // find the biggest power of 2 wrt FRAME_WIDTH
        int max_power = log(FRAME_WIDTH) / log(2);
        mu_0 = new double[max_power - 1];
        for (int i = 0; i <= max_power - 2; i++)
        {
            mu_0[i] = sqrt(2) / pow(2, i);
        }

        gabor_filtered_imgs = new cv::Mat[max_power * max_power - 1];
    }
    
    

 void SimulatedAnnealing::init(cv::Mat& img)
    {
        ROS_INFO("Convert the image to Luv space!");
        cv::cvtColor(img, img_, CV_RGB2Luv); // convert the image to luv space, important !!
        ROS_INFO("Convert the input image to grayscale!");
        //cv::cvtColor(img_gray_, img, CV_RGB2GRAY);
        
        

        temperature_ = Config::initial_temperature;

        prob_ = cv::Mat::ones(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
        prob_.setTo(cv::Scalar(255));

        output_= cv::Mat::ones(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);
        output_.setTo(cv::Scalar(255, 255, 255));

        random_image(prob_);
        convert(prob_, output_);

        prepareTexture(); //intialize vairables to compute the texture features

        c_.init();
    }

    void SimulatedAnnealing::run()
    {
        do
        {
            delta_global_energy_ = 0;

            // k avoids the influence by neighthood 
            for (int k = 0; k < 2; k++)
            {
                for (int i = 0; i < prob_.rows; i++)
                {
                    for (int j = k; j < prob_.cols; j += 2)
                    {
                        new_class_ = uint_dist_(eng_) % NB_COLORS;
                        delta_ = c_.compute(img_, i, j, new_class_, prob_) - c_.compute(img_, i, j, prob_.at<uchar>(i, j), prob_);
                                    
                        if ((delta_ <= 0.) || (exp(-delta_ / temperature_) >= ureal_dist_(eng_)))
                        {
                            delta_global_energy_ += fabs(delta_);
                            prob_.at<uchar>(i, j) = new_class_;
                        }
                    }
                }
            }

            convert(prob_, output_);
            temperature_ *= Config::temperature_decrease;
        }
        while(delta_global_energy_ > Config::min_change);

    	ROS_INFO("output segmented image");
        img_ = output_;

        //ROS_INFO("Conver the image back to RGB space !");
        //cv::cvtColor(img_, img_, CV_Luv2BGR); // convert the image back to rgb
        
    }

    bool SimulatedAnnealing::initCallback(markov_segm::Init::Request &req, markov_segm::Init::Response &res)
    {
        image_path_ = ros::package::getPath("markov_segm") + "/img/"; 
        image_name_ = req.image_name;
        cv::Mat img = cv::imread(image_path_ + image_name_);
        
        ROS_INFO("Read image and initialize class members !");
        init(img);
        return (res.status = true);
    }

    bool SimulatedAnnealing::runCallback(markov_segm::Run::Request &req, markov_segm::Run::Response &res)
    {
        
        run();

        cv::imwrite(image_path_ + "segmented_" + image_name_, img_);

        return (res.status = true);
    }
}

/*
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "markov_segmentation_code");
    ros::NodeHandle nh("markov_segm");

    //markov_segm::SimulatedAnnealing(nh);
    ros::spin();

    return 0;

}
*/




