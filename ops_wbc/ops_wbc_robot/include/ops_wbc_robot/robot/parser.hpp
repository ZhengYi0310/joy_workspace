#ifndef __OPS_WBC_ROBOT_PARSER_HPP
#define __OPS_WBC_ROBOT_PARSER_HPP

#include <fstream>
#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "ops_wbc_robot/robot/robot.hpp"

namespace ops_wbc_robot
{
    namespace yaml_tag
    {
        static const std::string ROBOT_NAME     = "name";
        static const std::string ROBOT_XYZ      = "xyz";
        static const std::string ROBOT_RPY      = "rpy";
        static const std::string WORLD_FRAME    = "world_frame";
        static const std::string MANIPULATORS   = "manipulators";
        static const std::string MNP_NAME       = "name";
        static const std::string LINKS          = "links";
        static const std::string LINK_NAME      = "name";
        static const std::string PARENT         = "parent";
        static const std::string JOINT_TYPE     = "joint_type";
        static const std::string LINK_XYZ       = "xyz_in_parent";
        static const std::string LINK_RPY       = "rpy_in_parent";
        static const std::string MASS           = "mass";
        static const std::string INERTIA_MATRIX = "inertia_matrix_in_com";
        static const std::string CENTER_OF_MASS = "com_in_link";
        static const std::string VX_MAX         = "vx_max";
        static const std::string VY_MAX         = "vy_max";
        static const std::string VZ_MAX         = "vy_max";
        static const std::string VROLL_MAX      = "vroll_max";
        static const std::string VPITCH_MAX     = "vpitch_max";
        static const std::string VYAW_MAX       = "vyaw_max";
        static const std::string Q_MIN          = "q_min";
        static const std::string Q_MAX          = "q_max";
        static const std::string DQ_MAX         = "dq_max";
        static const std::string TAU_MAX        = "tau_max";
        static const std::string INIT_Q         = "init_q";
        static const std::string DIFFERENTIATOR                  = "differentiator";
        static const std::string DIFFERENTIATOR_UPDATE_RATE      = "update_rate";
        static const std::string DIFFERENTIATOR_CUTOFF_FREQUENCY = "cutoff_frequency";
        static const std::string MACRO_MANIPULATOR_DOF = "macro_manipulator_dof";
    }

    class Parser 
    {
        public:
            explicit Parser() = default;
            void load(const std::string& path, const RobotPtr& robot);

        private:
            void loadRobotInfo(const RobotPtr& robot);
            void loadManipulator(const RobotPtr& robot);
            void loadLinks(const YAML::Node& node, const ManipulatorPtr& mnp);
            void loadVector3d(const YAML::Node& node, const std::string& tag, Eigen::Vector3d& v);
            void loadMatrix3d(const YAML::Node& node, const std::string& tag, Eigen::Matrix3d& m);
            void setLinkToManipulator(const std::map<std::string, double>& init_q, const ManipulatorPtr& mnp);
            void computeTotalDOF(const RobotPtr& robot);
            void checkTag(const YAML::Node& node, const std::string& tag, const std::string& func);
            std::ifstream ifs_;
            YAML::Node node_;
    };
    using ParserPtr = std::shared_ptr<Parser>;
}

#endif // __OPS_WBC_ROBOT_PARSER_HPP
