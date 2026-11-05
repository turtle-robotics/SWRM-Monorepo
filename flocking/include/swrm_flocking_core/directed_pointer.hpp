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
         * Allocates the managed object and the mutex that guards it.
         *
         * @param args Arguments forwarded to the constructor of `T`.
         */
        template<typename... Args>
        directed_pointer(Args&&... args) : 
                ptr(std::make_shared<T>(std::forward<Args>(args)...)), 
                mutex(std::make_shared<std::shared_mutex>()) {}

    public:
        /**
         * Copy construction is disabled so the pointer has one owner.
         */
        directed_pointer(const directed_pointer&) = delete;
        /**
         * Takes the object reference from another pointer and empties it.
         *
         * @param other Pointer to move from.
         */
        directed_pointer(directed_pointer&& other) noexcept : ptr(other.ptr) {
            other.ptr = nullptr;
        }
        
        /**
         * Copy assignment is disabled so the pointer has one owner.
         */
        directed_pointer& operator=(const directed_pointer&) = delete;
        /**
         * Takes over the object and mutex of another pointer.
         *
         * Assigning into a pointer that still has readers would leave those
         * readers guarding the wrong object, so this throws and leaves both
         * pointers unchanged. Readers of the source stay valid because this
         * pointer adopts its object and mutex.
         *
         * @param other Pointer to move from.
         * @return Reference to this pointer.
         * @throws std::runtime_error If this pointer still has readers.
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
         * Creates a reader that shares the object and its mutex.
         *
         * @return Reader pointing at the same object.
         */
        directed_reader<T> make_reader() const {
            return directed_reader{ptr, mutex};
        }

        /**
         * Locks the object for exclusive access.
         *
         * Waits until every reader has released its shared lock.
         *
         * @return Guard holding the unique lock.
         */
        directed_guard<T, std::unique_lock<std::shared_mutex>> lock() {
            return directed_guard<T, std::unique_lock<std::shared_mutex>>(ptr, std::unique_lock<std::shared_mutex>(*mutex));
        }

        /**
         * Counts the readers that currently share the object.
         *
         * @return Number of live readers.
         */
        unsigned int reader_count() const {
            return this->ptr.use_count() - 1;
        }
        
        /**
         * Releases this pointer's reference to the object.
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
         * Binds a reader to an existing object and its mutex.
         *
         * @param p Pointer to the shared object.
         * @param m Mutex guarding the object.
         */
        directed_reader(const std::shared_ptr<T>& p, std::shared_ptr<std::shared_mutex> m) : ptr(p), mutex(m) {}

    public:
        /**
         * Copies another reader's reference to the object.
         *
         * @param other Reader to copy.
         */
        directed_reader(const directed_reader& other) : ptr(other.ptr), mutex(other.mutex) {}
        /**
         * Moves the references out of another reader.
         *
         * @param other Reader to move from.
         */
        directed_reader(directed_reader&& other) noexcept : 
                ptr(std::move(other.ptr)), 
                mutex(std::move(other.mutex)) {}

        /**
         * Rebinds this reader to the object of another reader.
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
         * Moves the references out of another reader into this one.
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
         * Several readers can hold this lock at once, and it waits while a
         * writer holds the exclusive lock.
         *
         * @return Guard holding the shared lock.
         */
        directed_guard<const T, std::shared_lock<std::shared_mutex>> lock() const {
            return directed_guard<const T, std::shared_lock<std::shared_mutex>>(ptr, std::shared_lock<std::shared_mutex>(*mutex));
        }

        /**
         * Returns how many shared owners the object has.
         *
         * The count includes the directed pointer that created this reader and
         * every live reader.
         *
         * @return Shared owner count.
         */
        unsigned int use_count() const {
            return this->ptr.use_count();
        }
        
        /**
         * Releases this reader's reference to the object.
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
         * Binds the guard to an object and an acquired lock.
         *
         * @param p Pointer to the guarded object.
         * @param l Lock held for the lifetime of the guard.
         */
        directed_guard(std::shared_ptr<T> p, Lock&& l) : ptr(p), lock(std::move(l)) {}

    public:
        /**
         * Copy construction is disabled so each guard owns its lock.
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
         * Copy assignment is disabled so each guard owns its lock.
         */
        directed_guard& operator=(const directed_guard&) = delete;
        /**
         * Move assignment is disabled so each guard keeps its lock.
         */
        directed_guard& operator=(directed_guard&& other) = delete;

        /**
         * Returns a reference to the guarded object.
         *
         * @return Reference to the object.
         */
        T& operator->() {
            return *ptr;
        }

        /**
         * Dereferences the guard.
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
     * Creates a directed pointer that owns a new object.
     *
     * The arguments are forwarded to the constructor of `T`. The pointer
     * allocates the object and the mutex that guards it.
     *
     * @tparam T Type of the object to create.
     * @tparam Args Constructor argument types, deduced from the call.
     * @param args Arguments forwarded to the constructor of `T`.
     * @return Directed pointer owning the new object.
     */
    template <typename T, typename... Args>
    directed_pointer<T> make_directed(Args&&... args) {
        return directed_pointer<T>(std::forward<Args>(args)...);
    }
}


#endif // SWRM_FLOCKING_DIRECTED_POINTER_HPP
