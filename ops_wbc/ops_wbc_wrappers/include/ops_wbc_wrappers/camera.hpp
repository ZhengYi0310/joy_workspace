#ifndef __OPS_WBC_WRAPPERS_CAMERA_HPP
#define __OPS_WBC_WRAPPERS_CAMERA_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ops_wbc_wrappers
{
    class Camera
    {
        public:
            Camera(bool p, double fovy, double z_near, double z_far,
                   const Eigen::Vector3d& pp, const Eigen::Vector3d& cc, 
                   const Eigen::Vector3d& uu, double zoom_rate, double translate_rate, double rotate_rate);
            void look(int w, int h);
            void reset();

            void zoomin();
            void zoomout();
            void translate(int dx, int dy);
            void rotate(int dx, int dy);

            const Eigen::Vector3d& getPos() const 
            {
                return pos_;
            }

            const Eigen::Vector3d& getCenter() const 
            {
                return center_;
            }

            const Eigen::Vector3d& getUp() const 
            {
                return up_;
            }

        private:
            void calcOrthoParams(int w, int h);
            
            bool perspective_;
            
            double fovy_;
            double z_near_;
            double z_far_;
            
            double left_;
            double right_;
            double bottom_;
            double top_;

            Eigen::Vector3d pos_;
            Eigen::Vector3d center_;
            Eigen::Vector3d up_;

            Eigen::Vector3d pre_pos_;
            Eigen::Vector3d pre_center_;
            Eigen::Vector3d pre_up_;

            Eigen::Vector3d init_pos_;
            Eigen::Vector3d init_center_;
            Eigen::Vector3d init_up_;

            double zoom_rate_;
            double translate_rate_;
            double rotate_rate_;
    };
    typedef boost::shared_ptr<Camera> CameraPtr;
}

#endif // __OPS_WBC_WRAPPERS_CAMERA.HPP
