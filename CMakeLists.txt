cmake_minimum_required(VERSION 3.30)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(LIBATMC_DEVELOPMENT_MODE ON CACHE STRING "Build libcxxext in development mode.")
else()
    set(LIBATMC_DEVELOPMENT_MODE OFF CACHE STRING "Build libcxxext in development mode.")
endif()

# Config
include(CMakeDependentOption)
if(NOT BOARD_NAME)
    # Any of:
    #   - Hosted
    #   - STM32xxxx..., i.e. STM32H753ZI
    set(BOARD_NAME "Hosted")
endif()
cmake_dependent_option(LIBATMC_EXAMPLES "Build examples." ON "LIBATMC_DEVELOPMENT_MODE" OFF)

if(BOARD_NAME MATCHES "STM32")
    include(cmake/stm32_gcc.cmake)
endif()

project(libatmc)
enable_language(ASM C CXX)

# Compiler settings.

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_SCAN_FOR_MODULES FALSE)

if(NOT (CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU"))
    message(WARNING "Compiler set is not gcc, expect errors.")
endif()

include(libcxxext/cmake/clang_tidy.cmake)
include(cmake/stm32_prog.cmake)
include(cmake/target_specific.cmake)

# FreeRTOS

if(NOT BOARD_NAME STREQUAL "Hosted")
    add_library(freertos_config INTERFACE)
    target_include_directories(freertos_config SYSTEM INTERFACE ${RT_INCLUDE_DIRS})
    target_compile_definitions(freertos_config INTERFACE ${RT_DEFINITIONS} configUSE_PASSIVE_IDLE_HOOK=0 projCOVERAGE_TEST=0)

    target_compile_definitions(freertos_config INTERFACE ${definitions})
    target_compile_options(freertos_config INTERFACE ${options})

    set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)
    set(FREERTOS_PORT "GCC_ARM_CM7" CACHE STRING "" FORCE)

    add_subdirectory(freertos)
endif()

# Board Support
include(cmake/target_runtime_support.cmake)

# libcxxext

if(BOARD_NAME MATCHES "STM32")
    set(PLATFORM_SUPPORT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/core/platform")
endif()

add_subdirectory(libcxxext)
target_include_directories(sys.Threading PUBLIC "/")
target_link_libraries(sys.Threading PUBLIC rt)

add_subdirectory(core)
add_subdirectory(drivers)
add_subdirectory(examples)
