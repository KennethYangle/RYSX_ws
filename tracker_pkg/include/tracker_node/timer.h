#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <stdio.h>
#include <sys/time.h>

class Timer {
public :
    void tic() {
        gettimeofday(&time, NULL);
        time1 = time.tv_sec * 1000.0 + (time.tv_usec / 1000.0);
    }

    void toc(const char* task) {
        gettimeofday(&time, NULL);
        time2 = time.tv_sec * 1000.0 + (time.tv_usec / 1000.0);

        printf("%s took : %12.8f ms\n", task,  time2 - time1);
    }

    double toc() {
        gettimeofday(&time, NULL);
        time2 = time.tv_sec * 1000.0 + (time.tv_usec / 1000.0);

        return time2 - time1;
    }

private :
    struct timeval time;

    double time1;
    double time2;
};

#endif  // __GLOBAL_H
