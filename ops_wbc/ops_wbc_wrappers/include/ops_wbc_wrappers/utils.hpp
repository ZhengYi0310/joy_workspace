#ifndef __OPS_WBC_WRAPPERS_UTILS_HPP
#define __OPS_WBC_WRAPPERS_UTILS_HPP

#include <opencv2/opencv.hpp>

namespace ops_wbc_wrappers
{
    bool isEmpty(const cv::Mat& img);
    bool imshow(const std::string& name, const cv::Mat& img);
    char waitKey(double interval);
    void closing(const cv::Mat& src, cv::Mat& dst, unsigned int erodes, unsigned int dilates);
    void convertRGBToHSV(const cv::Mat& src, std::vector<cv::Mat>& hsv);
    void extractDesignatedArea(cv::Mat& gray, cv::Mat& dst, int min, int max);
    void extractDesignatedHueArea(cv::Mat& hue, cv::Mat& dst, int min, int max);
    void extractDesignatedSatArea(cv::Mat& sat, cv::Mat& dst, int min, int max);
    void extractDesignatedValArea(cv::Mat& val, cv::Mat& dst, int min, int max);
    void extractDesignatedHSVArea(std::vector<cv::Mat>& hsv, cv::Mat& dst,
                                  int h_min, int h_max,
                                  int s_min, int s_max,
                                  int v_min, int v_max,
                                  bool labeling, int erode_num, int dilate_num);
    void labeling(cv::Mat& src, cv::Mat& dst);
}

#endif // __OPS_WBC_WRAPPERS_UTILS_HPP
