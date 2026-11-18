#include <gtest/gtest.h>

#include <stdexcept>
#include <list>
#include <thread>
#include <latch>
#include <optional>

#include <swrm_flocking_core/directed_pointer.hpp>

TEST(TestDirectedPointer, FreeMemory) {
    // ASAN should catch any memory leaks after scope expires
    {
        swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);
    }

    // If no abort occurs, the test passes
    SUCCEED() << "Writer memory freed successfully without leaks.";
}

TEST(TestDirectedPointer, FreeReadWriteMemory) {
    // ASAN should catch any memory leaks after scope expires
    {
        swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);
        auto reader = ptr.make_reader();
    }

    // If no abort occurs, the test passes
    SUCCEED() << "Reader/Writer memory freed successfully without leaks.";
}

TEST(TestDirectedPointer, MoveAssignWithoutActiveReaders) {
    int check_value;
    {
        swrm::flocking::directed_pointer<int> ptr1 = swrm::flocking::make_directed<int>(42);
        swrm::flocking::directed_pointer<int> ptr2 = swrm::flocking::make_directed<int>(67);

        ptr1 = std::move(ptr2);
        check_value = *ptr1.lock();
    }
    
    // If no abort occurs, memory errors aren't present so just check that the value is correct
    EXPECT_EQ(check_value, 67) << "Move assignment without active readers failed.";
}

TEST(TestDirectedPointer, MoveAssignWithActiveReaders) {
    {
        swrm::flocking::directed_pointer<int> ptr1 = swrm::flocking::make_directed<int>(42);
        swrm::flocking::directed_pointer<int> ptr2 = swrm::flocking::make_directed<int>(67);

        auto reader = ptr1.make_reader();

        EXPECT_THROW(ptr1 = std::move(ptr2), std::runtime_error) << "Move assignment with active readers did not throw an exception.";
    }
    
    // Stack unwinding should free memory, so if no abort occurs, the test passes
    SUCCEED() << "Memory deallocated successfully after move assignment with active readers threw an exception.";
}

TEST(TestDirectedPointer, ReaderReadsValue) {
    swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);
    swrm::flocking::directed_reader<int> reader = ptr.make_reader();

    auto guard = reader.lock();
    EXPECT_EQ(*guard, 42) << "Reader did not read the correct value.";
}

TEST(TestDirectedPointer, WriterWritesValue) {
    swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);

    {
        auto guard = ptr.lock();
        *guard = 67;
    }

    auto reader = ptr.make_reader();
    auto guard = reader.lock();
    EXPECT_EQ(*guard, 67) << "Writer did not write the correct value.";
}

TEST(TestDirectedPointer, ReaderCount) {
    swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);
    EXPECT_EQ(ptr.reader_count(), 0) << "Initial reader count should be 0.";

    std::list<swrm::flocking::directed_reader<int>> readers;
    for (size_t i = 0; i < 5; ++i) {
        readers.push_back(ptr.make_reader());
        EXPECT_EQ(ptr.reader_count(), i + 1) << "Reader count should be " << (i + 1) << ".";
    }
}

TEST(TestDirectedPointer, UseCountOnReader) {
    swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);
    std::list<swrm::flocking::directed_reader<int>> readers;

    for (size_t i = 0; i < 5; ++i) {
        readers.push_back(ptr.make_reader());
        EXPECT_EQ(readers.back().use_count(), i + 2) << "Reader use count should be " << (i + 2) << ".";
    }
}

TEST(TestDirectedPointer, WriterLocksReader) {
    std::latch acquired_writer_lock(1);
    std::latch readers_starting(1);
    swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);
    swrm::flocking::directed_reader<int> reader = ptr.make_reader();
    int read_value = -1;

    {
        std::jthread writer_thread([&]() {
            auto writer_guard = ptr.lock();
            acquired_writer_lock.count_down();
            readers_starting.wait();
            // Wait for reader to attempt to acquire the lock before releasing it
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            *writer_guard = 67;
        });

        std::jthread reader_thread([&]() {
            acquired_writer_lock.wait();
            readers_starting.count_down();
            auto reader_guard = reader.lock();
            read_value = *reader_guard;
        });
    }
    EXPECT_EQ(read_value, 67) << "Reader did not read the updated value after writer lock was released.";
}

TEST(TestDirectedPointer, WriterLocksMultipleReaders) {
    std::latch acquired_writer_lock(1);
    std::latch readers_starting(2);
    swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);
    swrm::flocking::directed_reader<int> reader1 = ptr.make_reader();
    swrm::flocking::directed_reader<int> reader2 = ptr.make_reader();
    int read_value1 = -1;
    int read_value2 = -1;
    
    auto reader_thread_callback = [&](swrm::flocking::directed_reader<int>& reader, int& read_value) {
        acquired_writer_lock.wait();
        readers_starting.count_down();
        auto reader_guard = reader.lock();
        read_value = *reader_guard;
    };

    {
        std::jthread writer_thread([&]() {
            auto writer_guard = ptr.lock();
            acquired_writer_lock.count_down();
            readers_starting.wait();
            // Wait for readers to attempt to acquire the lock before releasing it
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            *writer_guard = 67;
        });

        std::jthread reader_thread1(reader_thread_callback, std::ref(reader1), std::ref(read_value1));
        std::jthread reader_thread2(reader_thread_callback, std::ref(reader2), std::ref(read_value2));
    }

    EXPECT_EQ(read_value1, 67) << "Reader 1 did not read the updated value after writer lock was released.";
    EXPECT_EQ(read_value2, 67) << "Reader 2 did not read the updated value after writer lock was released.";
}

TEST(TestDirectedPointer, ReaderLocksWriter) {
    std::latch acquired_reader_lock(1);
    std::latch writer_starting(1);
    swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);
    swrm::flocking::directed_reader<int> reader = ptr.make_reader();
    int read_value = -1;
    
    {
        std::jthread reader_thread([&]() {
            auto reader_guard = reader.lock();
            acquired_reader_lock.count_down();
            writer_starting.wait();
            // Wait for writer to attempt to acquire the lock before releasing it
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            read_value = *reader_guard;
        });

        std::jthread writer_thread([&]() {
            acquired_reader_lock.wait();
            writer_starting.count_down();
            auto writer_guard = ptr.lock();
            *writer_guard = 67;
        });
    }

    EXPECT_EQ(read_value, 42) << "Reader did not read the original value while holding the lock.";
}

TEST(TestDirectedPointer, MultipleReadersLockWriter) {
    std::latch acquired_reader_lock(2);
    std::latch writer_starting(1);
    swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);
    swrm::flocking::directed_reader<int> reader1 = ptr.make_reader();
    swrm::flocking::directed_reader<int> reader2 = ptr.make_reader();
    int read_value1 = -1;
    int read_value2 = -1;
    
    auto reader_thread_callback = [&](swrm::flocking::directed_reader<int>& reader, int& read_value) {
        auto reader_guard = reader.lock();
        acquired_reader_lock.count_down();
        writer_starting.wait();
        // Wait for writer to attempt to acquire the lock before releasing it
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        read_value = *reader_guard;
    };

    {
        std::jthread reader_thread1(reader_thread_callback, std::ref(reader1), std::ref(read_value1));
        std::jthread reader_thread2(reader_thread_callback, std::ref(reader2), std::ref(read_value2));

        std::jthread writer_thread([&]() {
            acquired_reader_lock.wait();
            writer_starting.count_down();
            auto writer_guard = ptr.lock();
            *writer_guard = 67;
        });
    }

    EXPECT_EQ(read_value1, 42) << "Reader 1 did not read the original value while holding the lock.";
    EXPECT_EQ(read_value2, 42) << "Reader 2 did not read the original value while holding the lock.";
}

TEST(TestDirectedPointer, ReaderOutlivesWriter) {
    std::optional<swrm::flocking::directed_reader<int>> reader = std::nullopt;
    {
        swrm::flocking::directed_pointer<int> ptr = swrm::flocking::make_directed<int>(42);
        reader.emplace(ptr.make_reader());
    }

    auto guard = reader->lock();
    EXPECT_EQ(*guard, 42) << "Reader did not read the original value after writer was destroyed.";
}
