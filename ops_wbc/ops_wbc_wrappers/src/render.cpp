/*************************************************************************
	> File Name: render.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 24 Jan 2018 04:53:30 PM PST
 ************************************************************************/

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <ops_wbc_wrappers/render.hpp>
#include <ros/ros.h>

using namespace ops_wbc_wrappers;

ParamPtr   Render::PARAM_;
CameraPtr  Render::CAMERA_;
LightPtr   Render::LIGHT_;
DisplayPtr Render::DISPLAY_;
MousePtr   Render::MOUSE_;

Render::Render(int& argc, char** argv)
{
    glutInit(&argc, argv);

    Render::PARAM_ = ParamPtr(new Param());
    Render::DISPLAY_ = DisplayPtr(new Display(Render::PARAM_->window_name, Render::PARAM_->window_h, Render::PARAM_->window_w, Render::PARAM_->color));
    Render::CAMERA_ = CameraPtr(new Camera(true, Render::PARAM_->fovy, Render::PARAM_->z_near, Render::PARAM_->z_far,
                                          Render::PARAM_->camera_pos, Render::PARAM_->camera_center, Render::PARAM_->camera_up,
                                          Render::PARAM_->zoom_rate, Render::PARAM_->translate_rate, Render::PARAM_->rotate_rate));
    Render::LIGHT_ = LightPtr(new Light(Render::PARAM_->light_pos[0], Render::PARAM_->ambient[0], Render::PARAM_->diffuse[0], Render::PARAM_->specular[0]));
    Render::MOUSE_ = MousePtr(new Mouse());
}

void Render::start(void (*display)())
{
    try
    {
        glutTimerFunc     (Render::PARAM_->fps, ops_wbc_wrappers::timer, 0);
        glutReshapeFunc   (ops_wbc_wrappers::reshape);
        glutKeyboardFunc  (ops_wbc_wrappers::keyboard);
        glutSpecialFunc   (ops_wbc_wrappers::specialKey);
        glutMouseFunc     (ops_wbc_wrappers::mouseClick);
        glutMotionFunc    (ops_wbc_wrappers::mouseDrag);
        glutDisplayFunc   (display);
        glutMainLoop();
    }
    catch(std::bad_alloc& e)
    {
        std::cerr << e.what() << std::endl;
        exit(1);
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        exit(1);
    }
    catch(...)
    {
        std::cerr << "ops_wbc_wrappers::Render::start : Unknown exception was thrown." << std::endl;
        exit(1);
    }
}

void Render::start()
{
    boost::mutex::scoped_lock(Render::PARAM_->mutex);

    Render::CAMERA_->look(Render::PARAM_->window_w, Render::PARAM_->window_h);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
}

void Render::end()
{
    glPopMatrix();
    glutSwapBuffers();
}

namespace ops_wbc_wrappers
{
    void reshape(int w, int h)
    {
        try
        {
            boost::mutex::scoped_lock(Render::PARAM_->mutex);
            Render::CAMERA_->look(w, h);
            Render::PARAM_->window_w = w;
            Render::PARAM_->window_h = h;
        }
        catch (ops_wbc_wrappers::Exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        catch(ops_wbc_wrappers::FatalException& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::bad_alloc& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(...)
        {
            std::cerr << "ops_wbc_wrappers::reshape : Unknown exception was thrown." << std::endl;
            exit(1);
        }
    }

    void timer(int value)
    {
        try
        {
            glutPostRedisplay();
            boost::mutex::scoped_lock(Render::PARAM_->mutex);
            glutTimerFunc(Render::PARAM_->fps, ops_wbc_wrappers::timer, 0);
        }
        catch(std::bad_alloc& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(...)
        {
            std::cerr << "ops_wbc_wrappers::timer : Unknown exception was thrown." << std::endl;
            exit(1);
        }
    }

    void specialKey(int key, int x, int y)
    {
        try
        {
            boost::mutex::scoped_lock(Render::PARAM_->mutex);
            switch(key)
            {
                case GLUT_KEY_UP:
                    Render::CAMERA_->zoomin();
                    break;
                case GLUT_KEY_DOWN:
                    Render::CAMERA_->zoomout();
                    break;
                default:
                    break;
            }
        }
        catch(ops_wbc_wrappers::Exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        catch(ops_wbc_wrappers::FatalException& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::bad_alloc& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(...)
        {
            std::cerr << "ops_wbc_wrappers::specialKey : Unknown exception was thrown." << std::endl;
            exit(1);
        }
    }

    void mouseClick(int button, int state, int x, int y)
    {
        try
        {
            boost::mutex::scoped_lock(Render::PARAM_->mutex);
            Render::MOUSE_->click(button, state, x, y);
        }
        catch(ops_wbc_wrappers::Exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        catch(ops_wbc_wrappers::FatalException& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::bad_alloc& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(...)
        {
            std::cerr << "ops_wbc_wrappers::mouseClick : Unknown exception was thrown." << std::endl;
            exit(1);
        }
    }

    void mouseDrag(int x, int y)
    {
        try
        {
            boost::mutex::scoped_lock(Render::PARAM_->mutex);
            Render::MOUSE_->drag(x, y);

            if(Render::MOUSE_->getButton() == GLUT_LEFT_BUTTON)
            {
                Render::CAMERA_->rotate(Render::MOUSE_->getdX(), Render::MOUSE_->getdY());
            }
            else if(Render::MOUSE_->getButton() == GLUT_RIGHT_BUTTON)
            {
                Render::CAMERA_->translate(Render::MOUSE_->getdX(), Render::MOUSE_->getdY());
            }
        }
        catch(ops_wbc_wrappers::Exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        catch(ops_wbc_wrappers::FatalException& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::bad_alloc& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(...)
        {
            std::cerr << "ops_wbc_wrappers::mouseDrag : Unknown exception was thrown." << std::endl;
            exit(1);
        }
    }

    void mousePassiveMotion(int x, int y)
    {
        try
        {
            boost::mutex::scoped_lock(Render::PARAM_->mutex);
        }
        catch(ops_wbc_wrappers::Exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        catch(ops_wbc_wrappers::FatalException& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::bad_alloc& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(...)
        {
            std::cerr << "ops_wbc_wrappers::mousePassiveMotion : Unknown exception was thrown." << std::endl;
            exit(1);
        }
    }

    void mouseWheel(int wheel_id, int direction, int x, int y)
    {
        try
        {
            boost::mutex::scoped_lock(Render::PARAM_->mutex);
            Render::MOUSE_->scroll(wheel_id, direction, x, y);
        }
        catch(ops_wbc_wrappers::Exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        catch(ops_wbc_wrappers::FatalException& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::bad_alloc& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
        catch(...)
        {
            std::cerr << "ops_wbc_wrappers::mouseWheel : Unknown exception was thrown." << std::endl;
            exit(1);
        }
    }
}
