#pragma once

#ifdef __clang__
    #define _clWarn_bad_offsetof "-Winvalid-offsetof"
    #define _clWarn_nontrivial_memcpy "-Wsuspicious-memaccess"
    #define _clWarn_unused_param "-Wunused-parameter"
    #define _clWarn_pedantic "-Wpedantic"
    #define _clWarn_terminate "-Wterminate"
    #define _clWarn_use_after_free "-Wblock-capture-autoreleasing"
#elifdef __GNUC__
    #define _clWarn_bad_offsetof "-Winvalid-offsetof"
    #define _clWarn_nontrivial_memcpy "-Wclass-memaccess"
    #define _clWarn_unused_param "-Wunused-parameter"
    #define _clWarn_pedantic "-Wpedantic"
    #define _clWarn_terminate "-Wterminate"
    #define _clWarn_use_after_free "-Wuse-after-free"
#else
    #define _clWarn_bad_offsetof ""
    #define _clWarn_nontrivial_memcpy ""
    #define _clWarn_unused_param ""
    #define _clWarn_pedantic ""
    #define _clWarn_terminate ""
    #define _clWarn_use_after_free ""
#endif

#define _clPragma_fwd(...) _Pragma(#__VA_ARGS__)
#define _push_nowarn(compilerWarning)                     \
    _Pragma("GCC diagnostic push");                       \
    _clPragma_fwd(GCC diagnostic ignored compilerWarning)
#define _pop_nowarn() _Pragma("GCC diagnostic pop")
