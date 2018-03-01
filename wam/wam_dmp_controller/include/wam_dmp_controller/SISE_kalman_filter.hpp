#ifndef __OPS_WBC_KALMAN_FILTER_KALMAN_FILTER_HPP
#define __OPS_WBC_KALMAN_FILTER_KALMAN_FILTER_HPP

#include <memory>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include "wam_dmp_controller/effective_mass_matrix.hpp"
#include "wam_dmp_controller/normal_distribution.hpp"

namespace wam_dmp_controller
{
    class SISE_KalmanFilter
    {
        public:
            void setRandomVariables(const NormalDistributionPtr& state_uncertainty,
                                    const NormalDistributionPtr& msr_noise);
            void InitializeLinearModel(const Eigen::MatrixXd& A,
                                       const Eigen::MatrixXd& B,
                                       const Eigen::MatrixXd& C,
                                       const Eigen::MatrixXd& W,
                                       const Eigen::MatrixXd& V);
            void InitializeEstimator(Eigen::MatrixXd x_mean_0, Eigen::MatrixXd P_xx_0);
            void updateCtrlMatrix(const Eigen::MatrixXd& coeff_of_ctrl_data);
            void estimate(const Eigen::MatrixXd& msr_data_curr, const Eigen::MatrixXd& msr_data_last);

            const NormalDistributionPtr& getState() const 
            {
                return state_;
            }

            const NormalDistributionPtr& getPredictedState() const 
            {
                return predicted_state_;
            }

            const NormalDistributionPtr& getInput() const 
            {
                return input_;
            }
        
        private:
            void checkMatrixSize(const Eigen::MatrixXd& ctrl_data, const Eigen::MatrixXd& msr_data);
            
            NormalDistributionPtr state_;
            NormalDistributionPtr input_;
            NormalDistributionPtr predicted_state_;
            NormalDistributionPtr state_uncertainty_;
            NormalDistributionPtr msr_noise_;

            double state_dim_;
            double measurement_dim_;
            double input_dim_;

            Eigen::MatrixXd u_mean_;
            Eigen::MatrixXd x_mean_;

            Eigen::MatrixXd A_;
            Eigen::MatrixXd B_;
            Eigen::MatrixXd C_;
            Eigen::MatrixXd P_;
            Eigen::MatrixXd G_;
            Eigen::MatrixXd K_;
            Eigen::MatrixXd P_xx_;
            Eigen::MatrixXd P_uu_;
            Eigen::MatrixXd P_ux_;
            Eigen::MatrixXd Q_;
            Eigen::MatrixXd Q_inverse_;
            Eigen::MatrixXd H_;
            Eigen::MatrixXd S_;
            Eigen::MatrixXd T_;
            Eigen::MatrixXd U_;
            Eigen::MatrixXd L_;
            
    };
    typedef boost::shared_ptr<SISE_KalmanFilter> KalmanFilterPtr;
}
#endif // __OPS_WBC_KALMAN_FILTER_KALMAN_FILTER_HPP
