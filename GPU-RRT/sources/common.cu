#include "common.h"
#include <device_launch_parameters.h>

void checkCUDAErrorFn(const char* msg, const char* file, int line) {
    cudaError_t err = cudaGetLastError();
    if (cudaSuccess == err) {
        return;
    }

    fprintf(stderr, "CUDA error");
    if (file) {
        fprintf(stderr, " (%s:%d)", file, line);
    }
    fprintf(stderr, ": %s: %s\n", msg, cudaGetErrorString(err));
    exit(EXIT_FAILURE);
}


namespace RRT {
    namespace Common {


        PerformanceTimer& timerCPU() {
            static PerformanceTimer timer;
            return timer;
        }

        PerformanceTimer& timerGPU() {
            static PerformanceTimer timer;
            return timer;
        }


        PerformanceTimer& timerpRRT() {
            static PerformanceTimer timer;
            return timer;
        }


    }
}
