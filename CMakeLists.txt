cmake_minimum_required(VERSION 3.30)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(LIBATMC_DEVELOPMENT_MODE ON CACHE STRING "Build libcxxext in development mode.")
else()
    set(LIBATMC_DEVELOPMENT_MODE OFF CACHE STRING "Build libcxxext in development mode.")
endif()

# Config
include(CMakeDependentOption)
if(NOT BOARD_NAME)
    set(BOARD_NAME "Hosted" CACHE STRING "Board name.") # i.e. STM32H753ZI
endif()
cmake_dependent_option(LIBATMC_EXAMPLES "Build examples." ON "LIBATMC_DEVELOPMENT_MODE" OFF)

if(BOARD_NAME MATCHES "STM32")
    include(cmake/stm32_gcc.cmake)
endif()

project(libatmc)
enable_language(ASM C CXX)

# Compiler settings.

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 26 REQUIRED)
set(CMAKE_CXX_SCAN_FOR_MODULES FALSE)

if(NOT (CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU"))
    message(WARNING "Compiler set is not gcc, expect errors.")
endif()

include(libcxxext/cmake/clang_tidy.cmake)
include(cmake/stm32_prog.cmake)

# Target specific configuration.

if(BOARD_NAME MATCHES "STM32")
    string(SUBSTRING ${BOARD_NAME} 0 7 RUNTIME_LIBRARY)
    set(RUNTIME_LIBRARY "${RUNTIME_LIBRARY}xx")
    string(SUBSTRING ${BOARD_NAME} 0 9 MCU_ID)
    set(MCU_ID "${MCU_ID}xx")
    message(STATUS "Targeting ${BOARD_NAME} (${MCU_ID}) with ${RUNTIME_LIBRARY} support libraries.")
elseif(BOARD_NAME STREQUAL "Hosted")
    message(STATUS "Targeting generic hosted platform.")
else()
    message(FATAL_ERROR "Unknown MCU runtime.")
endif()

if(BOARD_NAME MATCHES "STM32")
    file(GLOB RUNTIME_LINKER_SCRIPT_QUERY
        "stm32/${BOARD_NAME}*_FLASH.ld"
    )
    message(STATUS "Found linker script for runtime \"${RUNTIME_LINKER_SCRIPT_QUERY}\".")
    list(LENGTH RUNTIME_LINKER_SCRIPT_QUERY RUNTIME_LINKER_SCRIPT_QUERY_LENGTH)
    if(RUNTIME_LINKER_SCRIPT_QUERY_LENGTH EQUAL 0)
        message(FATAL_ERROR "No linker script found for ${BOARD_NAME}.")
    elseif(RUNTIME_LINKER_SCRIPT_QUERY_LENGTH GREATER 1)
        message(FATAL_ERROR "Multiple linker scripts found for ${BOARD_NAME}.")
    endif()
    list(GET RUNTIME_LINKER_SCRIPT_QUERY 0 RUNTIME_LINKER_SCRIPT)

    set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${RUNTIME_LINKER_SCRIPT}\"")
    set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -T \"${RUNTIME_LINKER_SCRIPT}\"")
    if(RUNTIME_LIBRARY STREQUAL "STM32H7xx")
        set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_CURRENT_SOURCE_DIR}/stm32/STM32H7xx_DMA.ld\"")
        set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -T \"${CMAKE_CURRENT_SOURCE_DIR}/stm32/STM32H7xx_DMA.ld\"")
    endif()

    file(GLOB RT_HEADERS
        "stm32/Drivers/${RUNTIME_LIBRARY}_HAL_Driver/Inc/*.h"
        "stm32/Core/Inc/*.h"
    )
    file(WRITE ${CMAKE_BINARY_DIR}/runtime_headers.h "#pragma once\n\n#ifdef __cplusplus\n#define register\n\nextern\"C\"\n{\n#endif\n\n")
    foreach(HEADER ${RT_HEADERS})
        get_filename_component(HEADER ${HEADER} NAME)
        if(NOT HEADER MATCHES "_template")
            file(APPEND ${CMAKE_BINARY_DIR}/runtime_headers.h "#include \"${HEADER}\" // IWYU pragma: export\n")
        endif()
    endforeach()
    file(APPEND ${CMAKE_BINARY_DIR}/runtime_headers.h "\n#ifdef __cplusplus\n}\n\n#undef register\n#endif\n")

    set(RT_INCLUDE_DIRS
        "${CMAKE_BINARY_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/stm32"
        "${CMAKE_CURRENT_SOURCE_DIR}/stm32/Drivers/CMSIS/Include"
        "${CMAKE_CURRENT_SOURCE_DIR}/stm32/Drivers/CMSIS/Device/ST/${RUNTIME_LIBRARY}/Include"
        "${CMAKE_CURRENT_SOURCE_DIR}/stm32/Drivers/${RUNTIME_LIBRARY}_HAL_Driver/Inc"
        "${CMAKE_CURRENT_SOURCE_DIR}/stm32/Core/Src"
        "${CMAKE_CURRENT_SOURCE_DIR}/stm32/Core/Inc"
        "${CMAKE_CURRENT_SOURCE_DIR}/freertos/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/core"
    )
    set(RT_DEFINITIONS "${MCU_ID}=1" "${RUNTIME_LIBRARY}=1")
    set(RT_DEFINITIONS ${RT_DEFINITIONS} "STM32=1" "HAL_ADC_MODULE_ENABLED")
elseif(BOARD_NAME STREQUAL "Hosted")
    file(WRITE ${CMAKE_BINARY_DIR}/runtime_headers.h "#pragma once\n")

    set(RT_INCLUDE_DIRS
        "${CMAKE_BINARY_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/generic"
    )
    set(RT_DEFINITIONS "HOSTED=1")
endif()

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

if(NOT BOARD_NAME STREQUAL "Hosted")
    file(GLOB RT_SRCS CONFIGURE_DEPENDS
        "stm32/Drivers/${RUNTIME_LIBRARY}_HAL_Driver/Inc/*.h"
        "stm32/Drivers/${RUNTIME_LIBRARY}_HAL_Driver/Src/*.c"
        "stm32/Drivers/BSP/${RUNTIME_LIBRARY}_Nucleo/*.h"
        "stm32/Drivers/BSP/${RUNTIME_LIBRARY}_Nucleo/*.c"
        "stm32/Core/Inc/*.h"
        "stm32/Core/Src/*.h"
        "stm32/Core/Src/*.c"
        "stm32/*.s"
        "stm32/board.cpp"
    )
    list(FILTER RT_SRCS EXCLUDE REGEX ".*_template.*")

    add_library(rt STATIC ${RT_SRCS})
    target_compile_definitions(rt PUBLIC ${RT_DEFINITIONS})
    target_include_directories(rt PUBLIC ${RT_INCLUDE_DIRS})
    target_link_libraries(rt PUBLIC freertos_kernel)
else()
    add_library(rt INTERFACE)
    target_sources(rt INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/generic/main.cpp")
    target_include_directories(rt INTERFACE ${RT_INCLUDE_DIRS})
    target_compile_definitions(rt INTERFACE ${RT_DEFINITIONS})
    target_link_libraries(rt INTERFACE stdc++exp)
endif()

# libcxxext

if(BOARD_NAME MATCHES "STM32")
    set(PLATFORM_SUPPORT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/core/platform")
endif()

add_subdirectory(libcxxext)
target_include_directories(sys.Threading PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(sys.Threading PUBLIC rt)

# Core Library

file(GLOB_RECURSE CORE_HEADERS CONFIGURE_DEPENDS
    "core/*.h"
)
list(FILTER CORE_HEADERS EXCLUDE REGEX "(rt_threading\..*|FreeRTOSConfig\.h)$")
file(GLOB_RECURSE CORE_SRCS CONFIGURE_DEPENDS
    "core/*.cpp"
)
list(FILTER CORE_SRCS EXCLUDE REGEX "rt_threading\..*$")

add_library(atmc.Core STATIC ${CORE_SRCS})
target_include_directories(atmc.Core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "core")
target_link_libraries(atmc.Core PUBLIC rt sys sys.Containers sys.Threading sys.BuildSupport.CompilerOptions sys.BuildSupport.WarningsAsErrors)

if(BOARD_NAME STREQUAL "Hosted")
    target_lint_clang_tidy(atmc.Core "-header-filter=\"^${CMAKE_CURRENT_SOURCE_DIR}/core/.*\"" ${CORE_HEADERS})
endif()

# Drivers Library

file(GLOB_RECURSE DRIVERS_HEADERS CONFIGURE_DEPENDS
    "drivers/*.h"
)
list(FILTER DRIVERS_HEADERS EXCLUDE REGEX "FreeRTOSConfig\.h$")

add_library(atmc.Drivers INTERFACE)
target_include_directories(atmc.Drivers INTERFACE "drivers")
target_link_libraries(atmc.Drivers INTERFACE atmc.Core sys.BuildSupport.CompilerOptions sys.BuildSupport.WarningsAsErrors)

if(BOARD_NAME STREQUAL "Hosted")
    target_lint_clang_tidy(atmc.Drivers "-header-filter=\"^${CMAKE_CURRENT_SOURCE_DIR}/drivers/.*\"" ${DRIVERS_HEADERS})
endif()

# Examples

file(GLOB EXAMPLES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp")

if(LIBATMC_EXAMPLES)
    foreach(EXAMPLE_FILE ${EXAMPLES})
        get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
        set(EXAMPLE_NAME "example_${EXAMPLE_NAME}")

        add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
        target_link_libraries(${EXAMPLE_NAME} PUBLIC atmc.Core atmc.Drivers sys.BuildSupport.CompilerOptions sys.BuildSupport.WarningsAsErrors)

        target_run_stm32_programmer(${EXAMPLE_NAME})
    endforeach()
endif()
