#ifndef __OPS_WBC_WRAPPERS_IMAGE_SUBSCRIBER_HPP
#define __OPS_WBC_WRAPPERS_IMAGE_SUBSCRIBER_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

namespace ops_wbc_wrappers
{
    class ImageSubscriber
    {
        public:
            ImageSubscriber(const std::string& topic, const std::string& encoding);
            bool updated();
            void copyTo(cv::Mat& img);
            void copyHeaderTo(std_msgs::Header& header);

        private:
            void imageCB(const sensor_msgs::ImageConstPtr& msg);
            
            boost::shared_ptr<image_transport::ImageTransport> it_;
            image_transport::Subscriber sub_;
            std::string encoding_;
            
            boost::mutex mutex_;
            bool updated_;

            cv_bridge::CvImagePtr cv_ptr_;
            std_msgs::Header header_;
    };
    typedef boost::shared_ptr<ImageSubscriber> ImageSubscriberPtr;
}

#endif // __OPS_WBC_WRAPPERS_IMAGE_SUBSCRIBER_HPP
