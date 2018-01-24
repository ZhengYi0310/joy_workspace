#ifndef __OPS_WBC_WRAPPERS_X_HAND_HPP
#define __OPS_WBC_WRAPPERS_X_HAND_HPP

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>
#include <ops_wbc_wrappers.hpp>
#include <ops_wbc_wrappers/x_deformable_object.hpp>

#include <ros/ros.h>

namespace ops_wbc_wrappers
{
    class XHandCore
    {
        public:
            XHandCore();
            XDeformableObject& getXHand()
            {
                return xhand_;
            }

        private:
            ros::NodeHandle nh_;
            XDeformableObject xhand_;
    };

    class XHand 
    {
        public:
            void rotateAllFingers(double deg);
            void rotate1stFinger(double deg);
            void rotate2ndFinger(double deg);
            void rotate3rdFinger(double deg);
            void rotate4thFinger(double deg);
            void rotate5thFinger(double deg);

            void display()
            {
                getLeftHand().display();
            }

            void displayWithoutShade()
            {
                getLeftHand().displayWithoutShade();
            }

        private:
            XDeformableObject& getLeftHand();
            ros::NodeHandle nh_;
    };

    class DeformableHandCore
    {
        public:
            DeformableHandCore(std::string file_name = "");
            XDeformableObject& getHand()
            {
                return hand_;
            }

        private:
            ros::NodeHandle nh_;
            XDeformableObject hand_;
    };

    class DeformableHand 
    {
        public:
            virtual ~DeformableHand() {}
            
            virtual void rotateAllFingers(double deg);
            virtual void rotate1stFinger(double deg) = 0;
            virtual void rotate2ndFinger(double deg) = 0;
            virtual void rotate3rdFinger(double deg) = 0;
            virtual void rotate4thFinger(double deg) = 0;
            virtual void rotate5thFinger(double deg) = 0;
            //virtual void rotateOrientation(Quaternion& quat);
            virtual void display() = 0;
            virtual void displayWithoutShade() = 0;
        
        private:
            virtual XDeformableObject& getRightHand() {}
            virtual XDeformableObject& getLeftHand() {}

        protected:
    };

    class RightHand : public DeformableHand
    {
        public:
            virtual void rotate1stFinger(double deg) override;
            virtual void rotate2ndFinger(double deg) override;
            virtual void rotate3rdFinger(double deg) override;
            virtual void rotate4thFinger(double deg) override;
            virtual void rotate5thFinger(double deg) override;
            RightHand(std::string file_name = "") : file_name_(file_name) {}
            void display();
            void displayWithoutShade();

        private:
            XDeformableObject& getRightHand();
            std::string file_name_;
    };

    class LeftHand : public DeformableHand
    {
        public:
            virtual void rotate1stFinger(double deg) override;
            virtual void rotate2ndFinger(double deg) override;
            virtual void rotate3rdFinger(double deg) override;
            virtual void rotate4thFinger(double deg) override;
            virtual void rotate5thFinger(double deg) override;
            LeftHand(std::string file_name = "") : file_name_(file_name) {}
            void display();
            void displayWithoutShade();

        private:
            XDeformableObject& getLeftHand();
            std::string file_name_;
    };

    typedef boost::shared_ptr<XHand> XHandPtr;
    typedef boost::shared_ptr<DeformableHand> DeformableHandPtr;
    typedef boost::shared_ptr<RightHand> RightHandPtr;
    typedef boost::shared_ptr<LeftHand> LeftHandPtr;
}

#endif // __OPS_WBC_WRAPPERS_X_HAND_HPP
