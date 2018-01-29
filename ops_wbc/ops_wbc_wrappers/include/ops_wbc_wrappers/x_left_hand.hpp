#ifndef __OPS_WBC_WRAPPERS_X_LEFT_HAND_HPP
#define __OPS_WBC_WRAPPERS_X_LEFT_HAND_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ops_wbc_wrappers/x_hand.hpp>

namespace ops_wbc_wrappers
{
    class XLeftHand
    {
        public:
            XLeftHand(const std::string& config_name, double scale);
            void display();
        private:
            double scale_;
            std::string config_name_;
            LeftHandPtr& getLeftHand();
            Eigen::MatrixXd pos_;
    };
    typedef boost::shared_ptr<XLeftHand> XLeftHandPtr;
}

#endif// __OPS_WBC_WRAPPERS_X_LEFT_HAND_HPP
