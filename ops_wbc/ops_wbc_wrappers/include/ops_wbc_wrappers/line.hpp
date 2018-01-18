#ifndef __OPS_WBC_WRAPPERS_LINE_HPP
#define __OPS_WBC_WRAPPERS_LINE_HPP

#include <vector>

namespace ops_wbc_wrappers
{
    class Line
    {
        public:
            Line(unsigned int max_size = 65535);
            void push_back(double x, double y, double z);
            void display();

        private:
            std::vector<double> x_;
            std::vector<double> y_;
            std::vector<double> z_;
            unsigned int max_size_;
    };
}

#endif
