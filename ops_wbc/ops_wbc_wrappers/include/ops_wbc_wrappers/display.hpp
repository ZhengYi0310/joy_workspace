#ifndef __OPS_WBC_WRAPPERS_DISPLAY_HPP
#define __OPS_WBC_WRAPPERS_DISPLAY_HPP

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace ops_wbc_wrappers
{
    class Display
    {
        public:
            Display(const std::string& name, int h, int w, const std::vector<int>& color);

        private:
            std::string name_;
            int h_;
            int w_;
            std::vector<int> color_;
    };

    typedef boost::shared_ptr<Display> DisplayPtr;
}

#endif // __OPS_WBC_WRAPPERS_DISPLAY_HPP
