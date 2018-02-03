#ifndef __OPS_WBC_ROBOT_MANIPULATOR_HPP
#define __OPS_WBC_ROBOT_MANIPULATOR_HPP

#include <map>
#include <vector>
#include <memory>
#include <Eigen/StdVector>
#include <ops_wbc_digital_filter/differentiator.hpp>
#include "ops_wbc_robot/definition.hpp"
#include <ops_wbc_robot/robot/link.hpp>

namespace ops_wbc_robot
{
    using VectorLinkPtr = std::vector<LinkPtr, Eigen::aligned_allocator<LinkPtr>>;
    using VectorMatrix4d = std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>;
    using VectorMatrix3d = std::vector<Eigen::Matrix3d>;
    using VectorMatrixXd = std::vector<Eigen::MatrixXd>;
    using VectorVector3d = std::vector<Eigen::Vector3d>;
    using MapMatrixXd = std::map<std::string,
                                 Eigen::MatrixXd,
                                 std::less<std::string>,
                                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::MatrixXd>>>;

    class Manipulator
    {
        public:
            explicit Manipulator(const std::string& name)
                : name_(name), link_(0), T_(0), T_abs_(0), C_abs_(0), Pin_(0), J_(0)
            {
            }

            void init(uint32_t dof, const Eigen::VectorXd& init_q);
            void update(const Eigen::VectorXd& q_msr);
            void update(const Eigen::VectorXd& q_msr, const Eigen::VectorXd& dq_msr);
            void computeJacobian();
            void computeMassMatrix();
            bool reached(const Eigen::VectorXd& qd, double threshold);
            bool hasLink(const std::string& name);
            void addLink(const LinkPtr& link);
            void reverseLink();

            inline void setDOF(uint32_t dof) {dof_ = dof;}
            inline void setMacroManipulatorDOF(uint32_t macro_dof) { macro_dof_ = macro_dof; }
            inline void setDifferentiatorUpdateRate(double update_rate) { update_rate_ = update_rate; }
            inline void setDifferentiatorCutoffFrequency(double cutoff_frequency) { cutoff_frequency_ = cutoff_frequency; }

            inline const std::string& getName() const {return name_;}
            inline const uint32_t getLinkNum() const {return link_.size();}
            inline const std::string& getLinkName(uint32_t idx) const {return link_[idx]->name;}
            inline const LinkPtr& getLink(uint32_t idx) const {return link_[idx];}
            inline const uint32_t getIndex(const std::string& name) {return name_to_idx_[name];}
            inline const uint32_t getDOF() const {return dof_;}
            inline const uint32_t getMarcoManipulatorDOF() const {return macro_dof_;}
            const Eigen::VectorXd& q() const {return q_;}
            const Eigen::VectorXd& dq() const {return dq_;}
            const Eigen::Matrix4d& getTransform(int32_t idx) const {return T_[idx];}
            const Eigen::Matrix4d& getTransformAbs(int32_t idx) const {return T_abs_[idx];}
            const VectorMatrixXd& getJacobian() const {return J_;}
            const Eigen::MatrixXd& getMassMatrix() const {return M_;}
            const Eigen::MatrixXd& getMassMatrixInv() const {return M_inv_;}
            void print();

        private:
            void computeForwardKinematics();
            void computeTabs();
            void computeCabs();
            void computeJacobian(int32_t idx, Eigen::MatrixXd& J);
            void computeVelocity();

            std::string name_ = "";
            uint32_t dof_ = 0;
            uint32_t macro_dof_ = 0;
            std::map<std::string, int32_t> name_to_idx_;

            bool updated_joint_ = false;
            double update_rate_ = 0.0;

            VectorLinkPtr link_;

            Eigen::VectorXd q_; // Generalized coordinates 
            Eigen::VectorXd dq_; // Velocity of generalized coordinates 

            VectorMatrix4d T_; // Relative transformation matrix associated with each link frame 
            VectorMatrix4d T_abs_; // Transformation matrix of i-th link w.r.t the base frame 
            
            VectorMatrix4d C_abs_; // Transformation matrix of i-th center of mass w.r.t the base frame 
            VectorVector3d Pin_; // End-effector position w.r.t link 

            VectorMatrixXd J_; // Basic Jacobian associated with the link jacobian 

            Eigen::MatrixXd M_; // Mass matrix 
            Eigen::MatrixXd M_inv_;

            ops_wbc_digital_filter::PseudoDifferentiatorPtr pseudo_differentiator_ = nullptr;
            double cutoff_frequency_ = 0.0;
    };
    using ManipulatorPtr = std::shared_ptr<Manipulator>;
}

#endif // __OPS_WBC_ROBOT_MANIPULATOR_HPP
