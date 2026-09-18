#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>

/* 1 = 打开调试打印，0 = 全部关掉 */
#define DEBUG_ENABLE    1

#if DEBUG_ENABLE
    #define DBG(...)    printf(__VA_ARGS__)
#else
    #define DBG(...)    ((void)0)
#endif

#endif
