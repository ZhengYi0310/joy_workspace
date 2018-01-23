#ifndef __OPS_WBC_WRAPPERS_PIPE_HPP
#define __OPS_WBC_WRAPPERS_PIPE_HPP

#include <boost/shared_ptr.hpp>
#include <ops_wbc_wrappers/simple_object.hpp>

namespace ops_wbc_wrappers
{
    class Pipe : public SimpleObject
    {
        public:
            Pipe(double h, double r_in, double r_out, unsigned int slices, unsigned int stacks);

        private:
            void displayImpl();

            GLuint model_list_;
            double h_;
            double r_in_;
            double r_out_;
            unsigned int slices_;
            unsigned int stacks_;
    };
}

#endif // _OPS_WBC_WRAPPERS_PIPE_HPP
