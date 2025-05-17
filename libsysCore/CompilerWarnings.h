#pragma once

#ifdef __clang__
    #define __clWarn_bad_offsetof "-Winvalid-offsetof"
    #define __clWarn_nontrivial_memcpy "-Wsuspicious-memaccess"
    #define __clWarn_unused_param "-Wunused-parameter"
    #define __clWarn_pedantic "-Wpedantic"
    #define __clWarn_terminate "-Wterminate"
    #define __clWarn_use_after_free "-Wblock-capture-autoreleasing"
#elifdef __GNUC__
    #define __clWarn_bad_offsetof "-Winvalid-offsetof"
    #define __clWarn_nontrivial_memcpy "-Wclass-memaccess"
    #define __clWarn_unused_param "-Wunused-parameter"
    #define __clWarn_pedantic "-Wpedantic"
    #define __clWarn_terminate "-Wterminate"
    #define __clWarn_use_after_free "-Wuse-after-free"
#else
    #define __clWarn_bad_offsetof ""
    #define __clWarn_nontrivial_memcpy ""
    #define __clWarn_unused_param ""
    #define __clWarn_pedantic ""
    #define __clWarn_terminate ""
    #define __clWarn_use_after_free ""
#endif

#define __clPragma_fwd(...) _Pragma(#__VA_ARGS__)
#define __push_nowarn(compilerWarning)                     \
    _Pragma("GCC diagnostic push");                        \
    __clPragma_fwd(GCC diagnostic ignored compilerWarning)
#define __pop_nowarn() _Pragma("GCC diagnostic pop")
