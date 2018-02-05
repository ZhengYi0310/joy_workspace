#ifndef __OPS_WBC_ROBOT_ROBOT_HPP
#define __OPS_WBC_ROBOT_ROBOT_HPP

#include <memory>
#include <map>
#include "ops_wbc_robot/definition.hpp"
#include "ops_wbc_robot/robot/manipulator.hpp"

namespace ops_wbc_robot
{
    using MapManipulatorPtr = std::map<std::string, ManipulatorPtr, std::less<std::string>, 
                                    Eigen::aligned_allocator<std::pair<const std::string, ManipulatorPtr>>>;
    class Robot 
    {
        public:
            explicit Robot(const std::string& robot_name) 
                : name_(robot_name),
                  pos_(Eigen::Vector3d::Zero()),
                  orient_(Eigen::Matrix3d::Identity()),
                  mnp_name_(0)
            { 
            }

            void computeJacobian();
            void computeJacobian(const std::string& mnp_name);
            void computeMassMatrix();
            void computeMassMatrix(const std::string& mnp_name);
            void update(const Eigen::VectorXd& q);
            
            void add(const ManipulatorPtr& mnp);
            bool reached(const std::string& mnp_name, const Eigen::VectorXd& qd, double threshold);
            
            void setDOF(uint32_t dof) {dof_ = dof;}
            void setMacroManipulatorDOF(uint32_t macro_dof) {macro_dof_ = macro_dof;}
            void setPosition(const Eigen::Vector3d& p) {pos_ = p;}
            void setOrientation(const Eigen::Quaternion<double>& q) {orient_ = q;}
            void setWorldFrame(const std::string& world) {world_ = world;}

            // Accessors 
            inline const std::string& getName() const {return name_;}
            inline const Eigen::Vector3d& getPosition() const {return pos_;}
            inline const Eigen::Quaternion<double>& getOrientation() const {return orient_;}
            inline const std::string& getWorldFrame() const {return world_;}
            inline const ManipulatorPtr& getManipulator(const std::string& name) {return mnp_[name];}
            inline const MapManipulatorPtr& getManipulator() const {return mnp_;}
            inline const std::vector<std::string>& getManipulatorName() const {return mnp_name_;}

            inline const Eigen::VectorXd& getJointPosition() const { return q_; }
            const Eigen::VectorXd& getJointPosition(const std::string& mnp_name);
            inline const Eigen::VectorXd& getJointVelocity() const { return dq_; }
            const Eigen::VectorXd& getJointVelocity(const std::string& mnp_name);
            const Eigen::MatrixXd& getJacobian(const std::string& mnp_name);
            const Eigen::MatrixXd& getJacobian(const std::string& mnp_name, const std::string& link_name);
            inline const Eigen::MatrixXd& getMassMatrix() const { return M_; }
            const Eigen::MatrixXd& getMassMatrix(const std::string& mnp_name);
            inline const Eigen::MatrixXd& getMassMatrixInv() const { return M_inv_; }
            const Eigen::MatrixXd& getMassMatrixInv(const std::string& mnp_name);
            inline const uint32_t getDOF() const { return dof_; }
            inline const uint32_t getDOF(const std::string& mnp_name);
            inline const uint32_t getMacroManipulatorDOF() const { return macro_dof_; }

        private:
            std::string name_ = "";
            std::string world_ = frame::WORLD;
            uint32_t dof_ = 0;
            uint32_t macro_dof_ = 0;

            Eigen::Vector3d pos_;
            Eigen::Quaternion<double> orient_;
            MapManipulatorPtr mnp_;
            std::vector<std::string> mnp_name_;

            Eigen::VectorXd q_;
            Eigen::VectorXd dq_;
            Eigen::MatrixXd M_;
            Eigen::MatrixXd M_inv_;
    };
    using RobotPtr = std::shared_ptr<Robot>;
}
#endif // __OPS_WBC_ROBOT_ROBOT_HPP
