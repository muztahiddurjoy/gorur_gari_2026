// A minimal check harness for the host tests (no framework to install).
#pragma once

#include <cstdio>

extern int host_failures;
extern int host_checks;

#define CHECK(cond)                                                          \
    do {                                                                     \
        host_checks++;                                                       \
        if (!(cond)) {                                                       \
            host_failures++;                                                 \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((double)(a) - (double)(b)) <= (tol))

#define RUN(test)                                                            \
    do {                                                                     \
        int before = host_failures;                                          \
        test();                                                              \
        std::printf("%s %s\n", host_failures == before ? "ok  " : "FAIL", #test); \
    } while (0)
