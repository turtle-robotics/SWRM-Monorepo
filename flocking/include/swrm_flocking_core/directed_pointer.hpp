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
        
        /**
         * Creates the managed object and the mutex that guards it.
         */
        template<typename... Args>
        directed_pointer(Args&&... args) : 
                ptr(std::make_shared<T>(std::forward<Args>(args)...)), 
                mutex(std::make_shared<std::shared_mutex>()) {}

    public:
        /**
         * Copy construction is deleted because each directed pointer has a
         * single owner.
         */
        directed_pointer(const directed_pointer&) = delete;
        /**
         * Moves the object reference out of another directed pointer.
         *
         * The other pointer no longer refers to the object after the move.
         *
         * @param other Pointer to move from.
         */
        directed_pointer(directed_pointer&& other) noexcept : ptr(other.ptr) {
            other.ptr = nullptr;
        }
        
        /**
         * Copy assignment is deleted.
         */
        directed_pointer& operator=(const directed_pointer&) = delete;
        /**
         * Takes over the object and the mutex of another directed pointer.
         *
         * The assignment throws while readers still hold references to the
         * current object, leaving both pointers unchanged.
         *
         * @param other Pointer to move from.
         * @return Reference to this pointer.
         * @throws std::runtime_error If this pointer has active readers.
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
    
        /**
         * Creates a reader for the managed object.
         *
         * The reader holds its own reference, so it stays valid after this
         * pointer is destroyed.
         *
         * @return Reader for the managed object.
         */
        directed_reader<T> make_reader() const {
            return directed_reader{ptr, mutex};
        }

        /**
         * Locks the managed object for exclusive access.
         *
         * The call blocks while any reader holds a shared lock.
         *
         * @return Guard that holds the exclusive lock.
         */
        directed_guard<T, std::unique_lock<std::shared_mutex>> lock() {
            return directed_guard<T, std::unique_lock<std::shared_mutex>>(ptr, std::unique_lock<std::shared_mutex>(*mutex));
        }

        /**
         * Returns the number of references held elsewhere.
         *
         * The reference owned by this pointer is not included.
         *
         * @return Number of references held elsewhere.
         */
        unsigned int reader_count() const {
            return this->ptr.use_count() - 1;
        }
        
        /**
         * Releases the reference held by this pointer.
         */
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

        /**
         * Binds the reader to an existing object and its mutex.
         */
        directed_reader(const std::shared_ptr<T>& p, std::shared_ptr<std::shared_mutex> m) : ptr(p), mutex(m) {}

    public:
        /**
         * Copies another reader's reference to the object and its mutex.
         *
         * @param other Reader to copy.
         */
        directed_reader(const directed_reader& other) : ptr(other.ptr), mutex(other.mutex) {}
        /**
         * Moves another reader's reference and mutex into this reader.
         *
         * @param other Reader to move from.
         */
        directed_reader(directed_reader&& other) noexcept : 
                ptr(std::move(other.ptr)), 
                mutex(std::move(other.mutex)) {}

        /**
         * Rebinds this reader to another reader's object and mutex.
         *
         * @param other Reader to copy.
         * @return Reference to this reader.
         */
        directed_reader& operator=(const directed_reader& other) {
            if (this == &other) return *this;

            this->ptr = other.ptr;
            mutex = other.mutex;
            return *this;
        }

        /**
         * Moves another reader's object reference and mutex into this reader.
         *
         * @param other Reader to move from.
         * @return Reference to this reader.
         */
        directed_reader& operator=(directed_reader&& other) noexcept {
            if (this == &other) return *this;

            this->ptr = std::move(other.ptr);
            mutex = std::move(other.mutex);
            return *this;
        }

        /**
         * Locks the object for reading.
         *
         * Several readers can hold the lock at once, and the call waits while a
         * writer holds the exclusive lock.
         *
         * @return Guard that holds the shared lock.
         */
        directed_guard<const T, std::shared_lock<std::shared_mutex>> lock() const {
            return directed_guard<const T, std::shared_lock<std::shared_mutex>>(ptr, std::shared_lock<std::shared_mutex>(*mutex));
        }

        /**
         * Returns the total reference count.
         *
         * The count includes the reference held by the directed pointer that
         * created this reader.
         *
         * @return Reference count.
         */
        unsigned int use_count() const {
            return this->ptr.use_count();
        }
        
        /**
         * Releases the reference held by this reader.
         */
        ~directed_reader() {}

        friend class directed_pointer<T>;
    };

    template <typename T, typename Lock>
    class directed_guard {
    private:
        std::shared_ptr<T> ptr;
        Lock lock;

        /**
         * Binds the guard to an object and an already held lock.
         *
         * The lock is released when the guard is destroyed.
         */
        directed_guard(std::shared_ptr<T> p, Lock&& l) : ptr(p), lock(std::move(l)) {}

    public:
        /**
         * Copy construction is deleted.
         */
        directed_guard(const directed_guard&) = delete;
        /**
         * Moves the object reference and the lock out of another guard.
         *
         * @param other Guard to move from.
         */
        directed_guard(directed_guard&& other) : 
                ptr(std::move(other.ptr)), 
                lock(std::move(other.lock)) {}

        /**
         * Copy assignment is deleted.
         */
        directed_guard& operator=(const directed_guard&) = delete;
        /**
         * Move assignment is deleted.
         */
        directed_guard& operator=(directed_guard&& other) = delete;

        /**
         * Provides member access to the guarded object.
         *
         * @return Reference to the guarded object.
         */
        T* operator->() {
            return ptr.get();
        }

        /**
         * Returns the guarded object.
         *
         * @return Reference to the guarded object.
         */
        T& operator*() {
            return *ptr;
        }
        
        template <typename U>
        friend class directed_pointer;
        template <typename U>
        friend class directed_reader;
    };
    
    /**
     * Creates a directed pointer that owns a newly constructed object.
     *
     * @tparam T Type of the object to create.
     * @tparam Args Constructor argument types.
     * @param args Arguments forwarded to the constructor of `T`.
     * @return Pointer that owns the new object.
     */
    template <typename T, typename... Args>
    directed_pointer<T> make_directed(Args&&... args) {
        return directed_pointer<T>(std::forward<Args>(args)...);
    }
}


#endif // SWRM_FLOCKING_DIRECTED_POINTER_HPP
