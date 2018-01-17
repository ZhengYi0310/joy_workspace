#ifndef __OPS_WBC_WRAPPERS_HSV_MASK_HPP
#define __OPS_WBC_WRAPPERS_HSV_MASK_HPP

#include <vector>
#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>
#include "ops_wbc_wrappers/hsv_mask_param.hpp"

namespace ops_wbc_wrappers
{
    class HSVMask
    {
        public:
            HSVMask(int h, int w);
            
            void set(const HSVMaskParamPtr& param, int idx);
            void add(const HSVMaskParamPtr& param);
            void getMask(std::vector<cv::Mat>& hsv, bool labeling, int erode_num, int dilate_num);
            const cv::Mat& get() const;

        private:
            std::vector<HSVMaskParamPtr> param_;
            cv::Mat mask_;
    };

    typedef boost::shared_ptr<HSVMask> HSVMaskPtr;
}

#endif // __OPS_WBC_WRAPPERS_HSV_MASK_HPP
