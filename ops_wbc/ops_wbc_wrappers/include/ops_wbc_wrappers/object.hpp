#ifndef __OPS_WBC_WRAPPERS_OBJECT_HPP
#define __OPS_WBC_WRAPPERS_OBJECT_HPP

#include <vector>
#include <GL/glut.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ops_wbc_wrappers/material.hpp"

namespace ops_wbc_wrappers
{
    class Object
    {
        public:
            Object();
            
            void display();
            bool haveEmptyParam();
            
            bool assign_normal_to_vertex_; // if false, assign normal to face 
            bool assign_material_to_vertex_;

            class Offset
            {
                public:
                    Offset() : euler_zyx(Eigen::Vector3d::Zero()), translation(Eigen::Vector3d::Zero())
                    {}
                    Eigen::Vector3d euler_zyx;
                    Eigen::Vector3d translation;
            };

            class Motion
            {
                public:
                    Motion() : euler_zyx(Eigen::Vector3d::Zero()), translation(Eigen::Vector3d::Zero())
                    {}
                    Eigen::Vector3d euler_zyx;
                    Eigen::Vector3d translation;
            };

            Offset offset;
            Motion motion;

            std::vector<Eigen::Vector3d> vertex;
            std::vector<std::vector<unsigned int> > v_idx;
            std::vector<Eigen::Vector3d> normal;
            std::vector<std::vector<unsigned int> > n_idx;

            std::vector<MaterialPtr> material;
            std::vector<unsigned int> material_assignment;

            std::string name;
            boost::shared_ptr<Object> parent;
            boost::shared_ptr<Object> child;

            GLuint model_list;
    };
    typedef boost::shared_ptr<Object> ObjectPtr;
}
#endif // __OPS_WBC_WRAPPERS_OBJECT_HPP
