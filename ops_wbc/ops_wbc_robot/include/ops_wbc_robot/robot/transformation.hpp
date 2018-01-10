#ifndef __OPS_WBC_ROBOT_TRASNFORMATION_HPP
#define __OPS_WBC_ROBOT_TRASNFORMATION_HPP

#include <memory>
#include <Eigen/Dense>

namespace ops_wbc_robot
{
    class Transformation
    {
        public:
            explicit Transformation() : T_(Eigen::Matrix4d::Identity()), axis_(Eigen::Vector3d::UnitX()) {}
            virtual ~Transformation() {}
            virtual const Eigen::Matrix4d& T(double q) = 0;
            virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T) = 0;
            virtual Eigen::Vector3d& axis()
            {
                return axis_;
            }

        protected:
            Eigen::Matrix4d T_;
            Eigen::Vector3d axis_;
    };
    using TransformationPtr = std::shared_ptr<Transformation>;

    class Fixed : public Transformation
    {
        public:
            virtual const Eigen::Matrix4d& T(double q) override
            {
                return T_;
            }
            virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T)
            {
            }
    };

    class RevoluteX : public Transformation
    {
        public:
            explicit RevoluteX();
            virtual const Eigen::Matrix4d& T(double q) override;
            virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T) override;
        private:
            Eigen::Matrix3d R_;
    };

    class RevoluteY : public Transformation
    {
        public:
            explicit RevoluteY();
            virtual const Eigen::Matrix4d& T(double q) override;
            virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T) override;
        private:
            Eigen::Matrix3d R_;
    };

    class RevoluteZ : public Transformation
    {
        public:
            explicit RevoluteZ();
            virtual const Eigen::Matrix4d& T(double q) override;
            virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T) override;
        private:
            Eigen::Matrix3d R_;
    };

    class PrismaticX : public Transformation
    {
        public:
            explicit PrismaticX();
            virtual const Eigen::Matrix4d& T(double q) override;
            virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T) override;
    };

    class PrismaticY : public Transformation
    {
        public:
            explicit PrismaticY();
            virtual const Eigen::Matrix4d& T(double q) override;
            virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T) override;
    };

    class PrismaticZ : public Transformation
    {
        public:
            explicit PrismaticZ();
            virtual const Eigen::Matrix4d& T(double q) override;
            virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T) override;
    };
}

#endif // __OPS_WBC_ROBOT_TRASNFORMATION_HPP
