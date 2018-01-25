#ifndef __OPS_WBC_WRAPPERS_RENDER_HPP
#define __OPS_WBC_WRAPPERS_RENDER_HPP

#include <boost/shared_ptr.hpp>
#include <ops_wbc_wrappers/param.hpp>
#include <ops_wbc_wrappers/display.hpp>
#include <ops_wbc_wrappers/camera.hpp>
#include <ops_wbc_wrappers/light.hpp>
#include <ops_wbc_wrappers/mouse.hpp>

namespace ops_wbc_wrappers
{
    class Render
    {
        public:
            static ParamPtr PARAM_;
            static CameraPtr CAMERA_;
            static LightPtr LIGHT_;
            static DisplayPtr DISPLAY_;
            static MoustPtr MOUSE_;

            Render(int& argc, char** argv);
            void start(void (*display)());
            static void start();
            static void end();

        private:
            Render() {}
            Render(const Render& render) {}
            Render& operator=(const Render& render) {}
    };

    void reshape(int w, int h);
    void timer(int value);
    void keyboard(unsigned char key, int x, int y);
    void specialKey(int key, int x, int y);
    void specialKeyUp(int key, int x, int y);
    void mouseClick(int button, int state, int x, int y);
    void mouseDrag(int x, int y);
    void mousePassiveMotion(int x, int y);
    void mouseWheel(int wheel_id, int direction, int x, int y);
    
    typedef boost::shared_ptr<Render> RenderPtr;
}

#endif // __OPS_WBC_WRAPPERS_RENDER_HPP
