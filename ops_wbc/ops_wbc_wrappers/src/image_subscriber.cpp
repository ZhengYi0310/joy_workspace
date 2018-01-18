/*************************************************************************
	> File Name: image_subscriber.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 17 Jan 2018 07:49:09 PM PST
 ************************************************************************/

#include <sensor_msgs/image_encodings.h>
#include "ops_wbc_wrappers/image_subscriber.hpp"
#include "ops_wbc_wrappers/exceptions.hpp"

using namespace ops_wbc_wrappers;

ImageSubscriber::ImageSubscriber(const std::string& topic, const std::string& encoding) : updated_(false), encoding_(encoding)
{
    ros::NodeHandle nh;
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));

    sub_ = it_->subscribe(topic, 1, &ImageSubscriber::imageCB, this);
}

bool ImageSubscriber::updated()
{
    boost::mutex::scoped_lock lock(mutex_);
    return updated_;
}

void ImageSubscriber::copyTo(cv::Mat& img)
{
    boost::mutex::scoped_lock lock(mutex_);
    img = cv_ptr_->image;
}

void ImageSubscriber::copyHeaderTo(std_msgs::Header& header)
{
    boost::mutex::scoped_lock lock(mutex_);
    header = header_;
}

void ImageSubscriber::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    boost::mutex::scoped_lock lock(mutex_);
    try 
    {
        cv_ptr_ = cv_bridge::toCvCopy(msg, encoding_);
        header_ = msg->header;
        updated_ = true;
    }
    catch(cv_bridge::Exception& e)
    {
        throw Exception("ImageSubscriber::imageCB", e.what());
    }
}
