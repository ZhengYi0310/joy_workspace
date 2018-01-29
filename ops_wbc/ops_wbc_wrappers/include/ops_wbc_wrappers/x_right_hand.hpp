#ifndef __OPS_WBC_WRAPPERS_X_RIGHT_HAND_HPP
#define __OPS_WBC_WRAPPERS_X_RIGHT_HAND_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include <ops_wbc_wrappers/x_hand.hpp>

namespace ops_wbc_wrappers
{
    class XRightHand
    {
        public:
            XRightHand(const std::string& config_name, double scale);
            void display();
        private:
            double scale_;
            std::string config_name_;
            RightHandPtr& getRightHand();
    };
    typedef boost::shared_ptr<XRightHand> XRightHandPtr;
}

#endif // __OPS_WBC_WRAPPERS_X_RIGHT_HAND_HPP
