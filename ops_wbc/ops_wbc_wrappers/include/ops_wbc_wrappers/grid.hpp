#ifndef __OPS_WBC_WRAPPERS_GRID_HPP
#define __OPS_WBC_WRAPPERS_GRID_HPP

#include <boost/shared_ptr.hpp>
#include "ops_wbc_wrappers/simple_object.hpp"

namespace ops_wbc_wrappers
{
    class Grid : public SimpleObject
    {
        public:
            Grid(unsigned int grid_num_x, unsigned int grid_num_y, double resolution);

        private:
            void displayImpl();
            
            unsigned int grid_num_x_;
            unsigned int grid_num_y_;
            double resolution_;
    };
}
#endif
