cmake_minimum_required(VERSION 3.30)

project(libcxxext)

set(LIBRARY_TYPE "STATIC" CACHE STRING "Build type of library.")

file(GLOB SYSCORE_SRCS CONFIGURE_DEPENDS
    "core/*.h"
    "core/*.cpp"
)
if (NOT PLATFORM_SUPPORT_RUNTIME_HEADER)
    set(PLATFORM_SUPPORT_RUNTIME_HEADER "${CMAKE_SOURCE_DIR}/core/rt_required_generic.h")
endif()
if (PLATFORM_SUPPORT_RUNTIME_LIBRARY)
    list(APPEND SYSCORE_SRCS ${PLATFORM_SUPPORT_RUNTIME_LIBRARY})
endif()

add_library(sys ${LIBRARY_TYPE} ${SYSCORE_SRCS})
target_include_directories(sys PUBLIC "/" "core" "libsysCore")
if(MSVC)
    target_compile_options(sys PUBLIC /FI "${PLATFORM_SUPPORT_RUNTIME_HEADER}")
else()
    target_compile_options(sys PUBLIC -include "${PLATFORM_SUPPORT_RUNTIME_HEADER}")
endif()
