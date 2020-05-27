#pragma once

#include <system_error>

#include <malloc.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>

inline void lock_and_prefault_mem(std::size_t size)
{
    // size: Size (in bytes) of locked memory pool

    // Lock all current and future pages from preventing of being paged to swap
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        throw std::system_error(errno, std::generic_category(), "Failed mlockall");

    // Turn off malloc trimming
    if (mallopt(M_TRIM_THRESHOLD, -1) != 1)
        throw std::runtime_error("Failed mallopt");

    // Turn off mmap usage
    if (mallopt(M_MMAP_MAX, 0) != 1)
        throw std::runtime_error("Failed mallopt");

    long int page_size = sysconf(_SC_PAGESIZE);
    char* buffer = static_cast<char*>(std::malloc(size));

    if (!buffer)
        throw std::runtime_error("Failed malloc");

    // Touch each page in this piece of memory to get it mapped into RAM for
    // performance improvement. Once the pagefault is handled, a page will be
    // locked in memory and never given back to the system
    for (unsigned i = 0; i < size; i += page_size)
        buffer[i] = 0;

    // Buffer is released but memory is locked to the process
    std::free(buffer);
}

inline void thread_set_sched_fifo_with_priority(pthread_t handle, int priority)
{
    // Enable realtime scheduling policy
    int policy;
    sched_param param;
    pthread_getschedparam(handle, &policy, &param);
    param.sched_priority = priority;

    if (pthread_setschedparam(handle, SCHED_FIFO, &param) != 0)
        throw std::system_error(errno, std::generic_category(), "Failed setschedparam");
}
