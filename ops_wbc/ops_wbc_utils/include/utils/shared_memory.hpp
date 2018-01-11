#ifndef __OPS_WBC_UTILS_SHARED_MEMORY_HPP
#define __OPS_WBC_UTILS_SHARED_MEMORY_HPP

#include <string>
#include <memory>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

using namespace boost::interprocess;

namespace ops_wbc_utils
{
    template<typename T>
    class SharedMemory
    {
        public:
            explicit SharedMemory(const std::string& name, bool remove = false) : name_(name)
            {
                size_ = sizeof(T);

                if (remove)
                {
                    shared_memory_object::remove(name.c_str());
                }

                try 
                {
                    shm_ = std::make_shared<shared_memory_object>(open_or_create, name_.c_str(), read_write);
                    shm_->truncate(size_);
                }

                catch(interprocess_exception&)
                {
                    shared_memory_object::remove(name.c_str());
                    shm_ = std::make_shared<shared_memory_object>(open_or_create, name_.c_str(), read_write);
                    shm_->truncate(size_);
                }

                region_ = std::make_shared<mapped_region>(*shm_, read_write);
                std::string mutex_name = name + "mutex";
                mutex_ = std::make_shared<named_mutex>(open_or_create, mutex_name.c_str());
                data_ = static_cast<T*>(region_->get_address());
            }

            void write(const T& val)
            {
                scoped_lock<named_mutex> lock(*mutex_);
                *data_ = val;
            }

            void read(T& val)
            {
                scoped_lock<named_mutex> lock(*mutex_);
                val = *data_;
            }

            const std::string& getName() const
            {
                return name_;
            }

            uint32_t getSize()
            {
                return size_;
            }

        private:
            using SharedMemoryObjectPtr = std::shared_ptr<shared_memory_object>;
            using MappedRegionPtr = std::shared_ptr<mapped_region>;
            using NamedMutexPtr = std::shared_ptr<named_mutex>;

            std::string name_;
            uint32_t size_;
            SharedMemoryObjectPtr shm_;
            MappedRegionPtr region_;
            NamedMutexPtr mutex_;
            T* data_;
    };

    template<typename T>
    using SharedMemoryPtr = std::shared_ptr<SharedMemory<T> >;
}

#endif // __OPS_WBC_UTILS_SHARED_MEMORY_HPP 
