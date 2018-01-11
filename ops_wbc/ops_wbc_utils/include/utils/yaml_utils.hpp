#ifndef __OPS_WBC_UTILS_YAML_UTILS_HPP
#define __OPS_WBC_UTILS_YAML_UTILS_HPP

#include <string>
#include <Eigen/Dense>
#include "utils/exception.hpp"

namespace ops_wbc_utils 
{
    class YAMLUtils
    {
        public:
            static std::string getVectorStr(const std::string& tag, const Eigen::MatrixXd& m)
            {
                std::stringstream msg;
                msg << tag << ": [";

                if (m.rows() == 0)
                {
                    msg << "]" << std::endl;
                    return msg.str();
                }

                if (m.cols() > 1)
                {
                    std::stringstream msg;
                    msg << "Eigen::MatrixXd::cols returned value which is bigger than 1." << std::endl << "YAMLUtils::VectorStr assumes a vertical vector.";
                    throw ops_wbc_utils::Exception("YAMLUtils::getVectorStr", msg.str());
                }

                msg << m.coeff(0, 0);
                for (uint32_t i = 0; i < m.rows(); i++)
                {
                    msg << ", " << m.coeff(i, 0);
                }
                msg << "]" << std::endl;

                return msg.str();
            }
    };
}
#endif // __OPS_WBC_UTILS_YAML_UTILS_HPP
