#pragma once

#ifndef THREAD_GROUP_H_MQSLWGKD
#define THREAD_GROUP_H_MQSLWGKD

// # define CPU_ZERO(cpusetp) __CPU_ZERO_S (sizeof (cpu_set_t), cpusetp)

// #define __CPU_ISSET_S(cpu, setsize, cpusetp) \
//   (__extension__      \
//    ({ size_t __cpu = (cpu);      \
//       __cpu / 8 < (setsize)      \
//       ? ((((const __cpu_mask *) ((cpusetp)->__bits))[__CPUELT (__cpu)]      \
//  & __CPUMASK (__cpu))) != 0      \
//       : 0; }))

// # define CPU_ISSET(cpu, cpusetp) __CPU_ISSET_S (cpu, sizeof (cpu_set_t), \
// cpusetp)

#include <thread>
#include <vector>

namespace rpc {
namespace detail {

class thread_group {
public:
    thread_group() {}
    thread_group(thread_group const &) = delete;

    void create_threads(std::size_t thread_count, std::function<void()> func) {
        for (std::size_t i = 0; i < thread_count; ++i) {
            threads_.push_back(std::thread(func));
        }

        // Set thread names
        for (std::size_t i = 0; i < thread_count; ++i) {
            std::string name = "rpc_thread_" + std::to_string(i);
            pthread_setname_np(threads_[i].native_handle(), name.c_str());
        }

        // Get accesible core mask
        cpu_set_t AvailableCpusMask;
        CPU_ZERO(&AvailableCpusMask);

        unsigned long num_accessible_cores = 0;

        std::FILE* cpuinfo = std::fopen("/home/idaretos/Control_AirSim/log/coreinfo.txt", "w");

        if (sched_getaffinity(0, sizeof(cpu_set_t), &AvailableCpusMask) == 0) {
            for (size_t i = 0; i < CPU_SETSIZE; i++) {
                if (CPU_ISSET(i, &AvailableCpusMask)) {
                    num_accessible_cores++;
                    if (cpuinfo) {
                        std::fprintf(cpuinfo, "Core %lu is available\n", i);
                    }
                }
            }
        }

        unsigned long min_cores_for_unreal = 8;
        unsigned long max_cores_for_unreal = 16;
        unsigned long cores_for_unreal = num_accessible_cores / 2;
        if (cores_for_unreal < min_cores_for_unreal) {
            if (num_accessible_cores > min_cores_for_unreal) {
                cores_for_unreal = min_cores_for_unreal;
            } else {
                cores_for_unreal = num_accessible_cores;
            }
        }
        if (num_accessible_cores - cores_for_unreal == 0) {
            cores_for_unreal -= 1;
        }

        if (cores_for_unreal > max_cores_for_unreal)
            cores_for_unreal = max_cores_for_unreal;

        if (cpuinfo) {
            std::fprintf(cpuinfo, "Number of accessible cores: %lu\n", num_accessible_cores);
            std::fprintf(cpuinfo, "Number of accessible cores for unreal: %lu\n", cores_for_unreal);
            std::fclose(cpuinfo);
        }

        if (num_accessible_cores > cores_for_unreal) {
            cpu_set_t AirSimCpusMask;
            CPU_ZERO(&AirSimCpusMask);
            for (size_t core = cores_for_unreal; core < num_accessible_cores; core++) {
                CPU_SET(core, &AirSimCpusMask);
            }

            for (auto &t : threads_) {
                pthread_setaffinity_np(t.native_handle(), sizeof(cpu_set_t), &AirSimCpusMask);
            }
        }
    }

    void join_all() {
        for (auto &t : threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
    }

    ~thread_group() { join_all(); }

private:
    std::vector<std::thread> threads_;
};

} /* detail */
} /* rpc  */

#endif /* end of include guard: THREAD_GROUP_H_MQSLWGKD */

