#ifndef __OPS_WBC_WRAPPERS_PARAM_HPP
#define __OPS_WBC_WRAPPERS_PARAM_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <GL/gl.h>
#include <Eigen/Dense>
#include <ops_wbc_wrappers/exceptions.hpp>

namespace ops_wbc_wrappers
{
    class Param
    {
        public:
            Param();

            boost::mutex mutex;
            
            std::string window_name;
            int window_h;
            int window_w;
            std::vector<int> color;

            double fps;

            bool orthogonal;
            double fovy;
            double z_near;
            double z_far;

            Eigen::Vector3d camera_pos;
            Eigen::Vector3d camera_center;
            Eigen::Vector3d camera_up;
            Eigen::Vector3d init_camera_pos;
            Eigen::Vector3d init_camera_center;
            Eigen::Vector3d init_camera_up;

            double zoom_rate;
            double translate_rate;
            double rotate_rate;

            int light_num;
            std::vector<Eigen::Vector4d> light_pos;
            std::vector<Eigen::Vector4d> ambient;
            std::vector<Eigen::Vector4d> diffuse;
            std::vector<Eigen::Vector4d> specular;

        private:
            template<class T>
            void checkLowerBorder(T val, T min, const std::string& name)
            {
                if (val < min)
                {
                    std::stringstream msg;
                    msg << name << "(= " << val << ") should be larger than " << min << ".";
                    throw Exception("Param::Param", msg.str());
                }      
            }

            template<class T>
            void checkUpperBorder(T val, T max, const std::string& name)
            {
                if(val > max)
                {
                    std::stringstream msg;
                    msg << name << "(= " << val << ") should be smaller than " << max << ".";
                    throw Exception("Param::Param", msg.str());
                }      
            }
    };
    typedef boost::shared_ptr<Param> ParamPtr;
}

#endif // __OPS_WBC_WRAPPERS_PARAM_HPP
