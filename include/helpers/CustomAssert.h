#pragma once

#define M_ASSERT(condition, format, ...)                       \
    do                                                         \
    {                                                          \
        if (!(condition))                                      \
        {                                                      \
            printf("[ASSERT in %s:%d]\n", __FILE__, __LINE__); \
            printf(format, ##__VA_ARGS__);                     \
            std::terminate();                                  \
        }                                                      \
    } while (false)
