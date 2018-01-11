#ifndef __OPS_WBC_UTILS_SCOPED_LOCK
#define __OPS_WBC_UTILS_SCOPED_LOCK

#include <memory>
#include <mutex>

namespace ops_wbc_utils
{
    class ScopedLock
    {
        public:
            explicit ScopedLock(std::mutex& mutex) : mutex_(mutex)
            {
                mutex_.lock();
            }

            ~ScopedLock()
            {
                mutex_.unlock();
            }
                        

        private:
            std::mutex mutex_;
    };
}

#endif // __OPS_WBC_UTILS_SCOPED_LOCK
