/*************************************************************************
	> File Name: cv_wrapper_test.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 29 Jan 2018 02:49:07 PM PST
 ************************************************************************/

#include <ros/ros.h>
#include <ops_wbc_wrappers/image_subscriber.hpp>
using namespace ops_wbc_wrappers;

class ImageProcessor
{
    public:
        ImageProcessor()
        {
            ros::NodeHandle nh;
            timer_ = nh.createTimer(ros::Duration(0.033333), &ImageProcessor::timerCB, this);

            sub_rgb_ = ImageSubscriberPtr(new ImageSubscriber("/camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8));
            sub_depth_ = ImageSubscriberPtr(new ImageSubscriber("/camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_16SC1));
        }

    private:
        void timerCB(const ros::TimerEvent& e)
        {
            cv::Mat rgb;
            cv::Mat depth;

            if (sub_rgb_->updated())
            {
                sub_rgb_->copyTo(rgb);
                cv::imshow("rgb", rgb);
            }

            if (sub_depth_->updated())
            {
                sub_depth_->copyTo(depth);
                cv::imshow("depth", depth);
            }
            cv::waitKey(2);
        }

        ros::Timer timer_;
        ImageSubscriberPtr sub_rgb_;
        ImageSubscriberPtr sub_depth_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cv_wrapper_test");
    ros::NodeHandle nh_;

    ImageProcessor image_processor;
    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    return 0;
    
}

