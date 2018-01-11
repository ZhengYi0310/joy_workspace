#ifndef __OPS_WBC_UTILS_YAML_LOADER_HPP
#define __OPS_WBC_UTILS_YAML_LOADER_HPP

#include<string>
#include<fstream>
#include<memory>
#include<yaml-cpp/yaml.h>
#include<Eigen/Dense>
#include "utils/exception.hpp"

namespace ops_wbc_utils
{
    class YAMLLoader
    {
        public:
            explicit YAMLLoader(const std::string& yaml)
            {
                ifs_.open(yaml.c_str());
                if (ifs_.fail())
                {
                    std::stringstream msg;
                    msg << "Could not open \"" << yaml << "\".";
                    throw ops_wbc_utils::Exception("YAMLLoader::YAMLLoader", msg.str());
                }
                doc_ = YAML::Load(ifs_);
            }
            
            template<class T>
            bool loadValue(const std::string& tag, T& dst)
            {
                if (!doc_[tag])
                {
                    return false;
                }

                try
                {
                    dst = doc_[tag].as<T>();
                }
                catch(YAML::Exception& e)
                {
                    throw ops_wbc_utils::Exception("YAMLLoader::loadValue", e.what());
                }

                return true;
            }

            template<class T>
            bool loadVector(const std::string& tag, std::vector<T>& dst)
            {
                if (!doc_[tag])
                {
                    return false;
                }

                try
                {
                    dst.resize(doc_[tag].size());
                    for (uint32_t i = 0; i < doc_[tag].size(); i++)
                    {
                        dst[i] = doc_[tag][i].as<T>();
                    }
                }
                catch(YAML::Exception& e)
                {
                    throw ops_wbc_utils::Exception("YAMLLoader::loadValue", e.what());
                }
                return true;
            }

            bool loadVector(const std::string& tag, Eigen::MatrixXd& dst)
            {
                if (!doc_[tag])
                {
                    return false;
                }

                try
                {
                    dst.resize(doc_[tag].size(), 1);
                    for (uint32_t i = 0; i < doc_[tag].size(); i++)
                    {
                        dst.coeffRef(i, 0) = doc_[tag][i].as<double>();
                    }
                }
                catch(YAML::Exception& e)
                {
                    throw ops_wbc_utils::Exception("YAMLLoader::loadValue", e.what());
                }
                return true;
            }

            bool loadMatrix(const std::string& tag, Eigen::MatrixXd& dst)
            {
                if (!doc_[tag])
                {
                    return false;
                }

                try
                {
                    uint32_t rows = doc_[tag].size();
                    if(rows == 0)
                    {
                        throw ops_wbc_utils::Exception("YAMLLoader::loadMatrix", "rows is zero.");
                    }
                    uint32_t cols = doc_[tag][0].size();
                    dst.resize(rows, cols);
                    for (uint32_t i = 0; i < rows; i++)
                    {
                        for (uint32_t j = 0; j < cols; j++)
                        {
                            dst.coeffRef(i, j) = doc_[tag][i][j].as<double>();
                        }
                    }
                }
                catch(YAML::Exception& e)
                {
                    throw ops_wbc_utils::Exception("YAMLLoader::loadValue", e.what());
                }
                return true;
                 
            }

        private:
            std::fstream ifs_;
            YAML::Node doc_;
    };
    using YAMLLoaderPtr = std::shared_ptr<YAMLLoader>;
}

#endif // __OPS_WBC_UTILS_YAML_LOADER_HPP 
