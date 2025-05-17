#pragma once

#define __fence_result_return(rValRef, out) \
    do                                      \
    {                                       \
        auto __result = rValRef;            \
        if (!__result)                      \
            return __result.takeError();    \
        out = __result.takeValue();         \
    }                                       \
    while (false)
#define __fence_result_co_return(rValRef, out) \
    do                                         \
    {                                          \
        auto __result = rValRef;               \
        if (!__result)                         \
            co_return __result.takeError();    \
        out = __result.takeValue();            \
    }                                          \
    while (false)
