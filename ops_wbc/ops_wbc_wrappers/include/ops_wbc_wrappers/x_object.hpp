#ifndef __OPS_WBC_WRAPPERS_X_OBJECT_HPP
#define __OPS_WBC_WRAPPERS_X_OBJECT_HPP

#include <string>
#include <fstream>
#include <map>
#include <boost/shared_ptr.hpp>
//#include <Eigen/Dense>
#include <ops_wbc_wrappers/object.hpp>

namespace ops_wbc_wrappers
{
    class XObject 
    {
        public:
            XObject(const std::string& name);
            ~XObject();
            void setTranslation(const std::string& key, double x, double y, double z);
            void setEulerZYX(const std::string& key, double rz, double ry, double rx);
            void display();

        private:
            XObject();
            XObject(const XObject& obj);
            XObject& operator=(const XObject& obj);
            
            void getNextLineOf(const std::string& str, std::string& dst);
            void loadMapKey();
            void loadTransformMatrix(const ObjectPtr& obj, const std::string& key);
            void loadVertex(const ObjectPtr& obj, const std::string& key);
            void loadVertexIndex(const ObjectPtr& obj, const std::string& key);
            void loadNormal(const ObjectPtr& obj, const std::string& key);
            void loadNormalIndex(const ObjectPtr& obj, const std::string& key);
            void loadMaterial(const ObjectPtr& obj, const std::string& key);
            void createDisplayList(const ObjectPtr& obj, const std::string& key);
            void calcOffset(const Eigen::Matrix4d& transform, Object::Offset& offset);
            void printMapKey();
            void printf(const Eigen::Matrix4d& mat);
            void printf(const Eigen::Vector3d& vec);

            Eigen::Matrix4d root_transform_;
            std::ifstream ifs_;
            std::vector<std::string> key_;
            std::map<std::string, ObjectPtr> object_;
    };

    typedef boost::shared_ptr<XObject> XObjectPtr;
}

#endif // __OPS_WBC_WRAPPERS_X_OBJECT_HPP
