#ifndef SWRM_FLOCKING_DIRECTED_POINTER_HPP
#define SWRM_FLOCKING_DIRECTED_POINTER_HPP

#include <utility>
#include <memory>
#include <shared_mutex>
#include <stdexcept>

namespace swrm::flocking {
    template <typename T>
    class directed_pointer;
    template <typename T, typename... Args>
    directed_pointer<T> make_directed(Args&&... args);
    template <typename T>
    class directed_reader;
    template <typename T, typename Lock>
    class directed_guard;

    template <typename T>
    class directed_pointer {
    private:
        std::shared_ptr<T> ptr;
        mutable std::shared_ptr<std::shared_mutex> mutex;
        
        template<typename... Args>
        directed_pointer(Args&&... args) : 
                ptr(std::make_shared<T>(std::forward<Args>(args)...)), 
                mutex(std::make_shared<std::shared_mutex>()) {}

    public:
        // Make directed pointer non-copyable but movable
        directed_pointer(const directed_pointer&) = delete;
        directed_pointer(directed_pointer&& other) noexcept : ptr(other.ptr) {
            other.ptr = nullptr;
        }
        
        directed_pointer& operator=(const directed_pointer&) = delete;
        /* 
         * Only move-assign to this if there are no active directed_readers on the current pointer
         * This is to prevent desynchronization of this pointer and its directed_readers
         * The directed_readers of the other pointer will be still valid since this pointer now holds the 
         * other pointer's memory address and mutex
         */
        directed_pointer& operator=(directed_pointer&& other) {
            if (this == &other) return *this;
            if (this->ptr.use_count() > 1) {
                throw std::runtime_error("Cannot move assign a directed_pointer with active directed_readers.");
            }
            this->ptr = std::move(other.ptr);
            this->mutex = std::move(other.mutex);
            return *this;
        }
    
        directed_reader<T> make_reader() const {
            return directed_reader{ptr, mutex};
        }

        directed_guard<T, std::unique_lock<std::shared_mutex>> lock() {
            return directed_guard<T, std::unique_lock<std::shared_mutex>>(ptr, std::unique_lock<std::shared_mutex>(*mutex));
        }

        unsigned int reader_count() const {
            return this->ptr.use_count() - 1;
        }
        
        // Not needed, but here for Rule of Five compliance
        ~directed_pointer() {}
        
        template <typename U, typename... Args>
        friend directed_pointer<U> make_directed(Args&&... args);
        friend class directed_reader<T>;
    };
    
    template <typename T>
    class directed_reader {
    private:
        std::shared_ptr<T> ptr;
        mutable std::shared_ptr<std::shared_mutex> mutex;

        directed_reader(const std::shared_ptr<T>& p, std::shared_ptr<std::shared_mutex> m) : ptr(p), mutex(m) {}

    public:
        directed_reader(const directed_reader& other) : ptr(other.ptr), mutex(other.mutex) {}
        directed_reader(directed_reader&& other) noexcept : 
                ptr(std::move(other.ptr)), 
                mutex(std::move(other.mutex)) {}

        directed_reader& operator=(const directed_reader& other) {
            if (this == &other) return *this;

            this->ptr = other.ptr;
            mutex = other.mutex;
            return *this;
        }

        directed_reader& operator=(directed_reader&& other) noexcept {
            if (this == &other) return *this;

            this->ptr = std::move(other.ptr);
            mutex = std::move(other.mutex);
            return *this;
        }

        directed_guard<const T, std::shared_lock<std::shared_mutex>> lock() const {
            return directed_guard<const T, std::shared_lock<std::shared_mutex>>(ptr, std::shared_lock<std::shared_mutex>(*mutex));
        }

        unsigned int use_count() const {
            return this->ptr.use_count();
        }
        
        // Not needed, but here for Rule of Five compliance
        ~directed_reader() {}

        friend class directed_pointer<T>;
    };

    template <typename T, typename Lock>
    class directed_guard {
    private:
        std::shared_ptr<T> ptr;
        Lock lock;

        directed_guard(std::shared_ptr<T> p, Lock&& l) : ptr(p), lock(std::move(l)) {}

    public:
        directed_guard(const directed_guard&) = delete;
        directed_guard(directed_guard&& other) : 
                ptr(std::move(other.ptr)), 
                lock(std::move(other.lock)) {}

        directed_guard& operator=(const directed_guard&) = delete;
        directed_guard& operator=(directed_guard&& other) = delete;

        T& operator->() {
            return *ptr;
        }

        T& operator*() {
            return *ptr;
        }
        
        template <typename U>
        friend class directed_pointer;
        template <typename U>
        friend class directed_reader;
    };
    
    template <typename T, typename... Args>
    directed_pointer<T> make_directed(Args&&... args) {
        return directed_pointer<T>(std::forward<Args>(args)...);
    }
}


#endif // SWRM_FLOCKING_DIRECTED_POINTER_HPP
