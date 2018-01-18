#ifndef __OPS_WBC_WRAPPERS_COORDINATE_HPP
#define __OPS_WBC_WRAPPERS_COORDINATE_HPP

#include "ops_wbc_wrappers/simple_object.hpp"

namespace ops_wbc_wrappers
{
    class Coordinate : public SimpleObject
    {
        public:
            Coordinate(double scale = 1.0, double length = 0.5);

        private:
            void displayImpl();

            double scale_;
            double length_;
    };
}

#endif // __OPS_WBC_WRAPPERS_COORDINATE_HPP
