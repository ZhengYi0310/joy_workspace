#ifndef __OPS_WBC_WRAPPERS_MOUSE_HPP
#define __OPS_WBC_WRAPPERS_MOUSE_HPP

#include <boost/shared_ptr.hpp>

namespace ops_wbc_wrappers
{
    class Mouse 
    {
        public:
            Mouse() : button_(0), state_(0), pre_x_(0), pre_y_(0), dx_(0), dy_(0)
            {}

            void click(int button, int state, int x, int y);
            void drag(int x, int y);
            void scroll(int wheel_id, int direction, int x, int y);

            const int getButton() const 
            {
                return button_;
            }

            const int getState() const 
            {
                return state_;
            }

            const int getdX() const 
            {
                return dx_;
            }

            const int getdY() const 
            {
                return dy_;
            }
        
        private:
            int button_;
            int state_;
            int pre_x_;
            int pre_y_;
            int dx_;
            int dy_;
    };
    typedef boost::shared_ptr<Mouse> MousePtr;
}
#endif // __OPS_WBC_WRAPPERS_MOUSE_HPP
