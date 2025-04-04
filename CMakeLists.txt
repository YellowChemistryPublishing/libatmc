cmake_minimum_required(VERSION 3.30)

include(cmake/stm32_gcc.cmake)

project(libatmcDrivers)
enable_language(ASM C CXX)

set(BOARD_NAME "STM32H753ZI")

# Compiler settings.

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 26 REQUIRED)
set(CMAKE_CXX_SCAN_FOR_MODULES FALSE)

add_compile_options(
    -Werror $<$<COMPILE_LANGUAGE:CXX>:-Wsuggest-override> # "-fdebug-prefix-map=${CMAKE_SOURCE_DIR}=."
    -fno-math-errno -funsafe-math-optimizations -fno-rounding-math -fno-signed-zeros -fno-trapping-math -fcx-limited-range -fexcess-precision=fast
    -foptimize-sibling-calls # Force symmetric transfer optimization for coroutines.
)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fno-signaling-nans) # -fopt-info-optall)
elseif (CMAKE_C_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-ffp-contract=fast $<$<COMPILE_LANGUAGE:C>:-include> $<$<COMPILE_LANGUAGE:C>:stddef.h>) # -Rpass-analysis=.*)
else()
    message(WARNING "Compiler set unrecognised. Some optimisations may not be available.")
endif()

# Target Specific

if (BOARD_NAME MATCHES "STM32")
    string(SUBSTRING ${BOARD_NAME} 0 7 RUNTIME_LIBRARY)
    set(RUNTIME_LIBRARY "${RUNTIME_LIBRARY}xx")
    string(SUBSTRING ${BOARD_NAME} 0 9 MCU_ID)
    set(MCU_ID "${MCU_ID}xx")
    message(STATUS "Targeting ${BOARD_NAME} (${MCU_ID}) with ${RUNTIME_LIBRARY} support libraries.")
else()
    message(FATAL_ERROR "Unknown MCU runtime.")
endif()

file(GLOB RUNTIME_LINKER_SCRIPT_QUERY
    "stm32/${BOARD_NAME}*_FLASH.ld"
)
message(STATUS "Found linker script for runtime \"${RUNTIME_LINKER_SCRIPT_QUERY}\".")
list(LENGTH RUNTIME_LINKER_SCRIPT_QUERY RUNTIME_LINKER_SCRIPT_QUERY_LENGTH)
if (RUNTIME_LINKER_SCRIPT_QUERY_LENGTH EQUAL 0)
    message(FATAL_ERROR "No linker script found for ${BOARD_NAME}.")
elseif (RUNTIME_LINKER_SCRIPT_QUERY_LENGTH GREATER 1)
    message(FATAL_ERROR "Multiple linker scripts found for ${BOARD_NAME}.")
endif()
list(GET RUNTIME_LINKER_SCRIPT_QUERY 0 RUNTIME_LINKER_SCRIPT)

set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${RUNTIME_LINKER_SCRIPT}\"")
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -T \"${RUNTIME_LINKER_SCRIPT}\"")

file(GLOB RT_HEADERS
    "stm32/Drivers/${RUNTIME_LIBRARY}_HAL_Driver/Inc/*.h"
    "stm32/Core/Inc/*.h"
)
file(WRITE ${CMAKE_BINARY_DIR}/runtime_headers.h "#pragma once\n\n#ifdef __cplusplus\n#define register\n\nextern\"C\"\n{\n#endif\n\n")
foreach(HEADER ${RT_HEADERS})
    get_filename_component(HEADER ${HEADER} NAME)
    if (NOT HEADER MATCHES "_template")
        file(APPEND ${CMAKE_BINARY_DIR}/runtime_headers.h "#include \"${HEADER}\"\n")
    endif()
endforeach()
file(APPEND ${CMAKE_BINARY_DIR}/runtime_headers.h "\n#ifdef __cplusplus\n}\n\n#undef register\n#endif\n")

set(RT_INCLUDE_DIRS 
    "${CMAKE_BINARY_DIR}"
    "${CMAKE_SOURCE_DIR}/stm32/Drivers/CMSIS/Include"
    "${CMAKE_SOURCE_DIR}/stm32/Drivers/CMSIS/Device/ST/${RUNTIME_LIBRARY}/Include"
    "${CMAKE_SOURCE_DIR}/stm32/Drivers/${RUNTIME_LIBRARY}_HAL_Driver/Inc"
    "${CMAKE_SOURCE_DIR}/stm32/Core/Src"
    "${CMAKE_SOURCE_DIR}/stm32/Core/Inc"
    "${CMAKE_SOURCE_DIR}/freertos/include"
    "${CMAKE_SOURCE_DIR}/core"
)
set(RT_DEFINITIONS "${MCU_ID}=1" "STM32=1")

# FreeRTOS

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE ${RT_INCLUDE_DIRS})
target_compile_definitions(freertos_config INTERFACE ${RT_DEFINITIONS} configUSE_PASSIVE_IDLE_HOOK=0 projCOVERAGE_TEST=0) # configNUMBER_OF_CORES=2

target_compile_definitions(freertos_config INTERFACE ${definitions})
target_compile_options(freertos_config INTERFACE ${options})

set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)
set(FREERTOS_PORT "GCC_ARM_CM7" CACHE STRING "" FORCE)

add_subdirectory(freertos)

# stm32 Runtime

file(GLOB RT_SRCS CONFIGURE_DEPENDS
    "stm32/Drivers/${RUNTIME_LIBRARY}_HAL_Driver/Inc/*.h"
    "stm32/Drivers/${RUNTIME_LIBRARY}_HAL_Driver/Src/*.c"
    "stm32/Drivers/BSP/${RUNTIME_LIBRARY}_Nucleo/*.h"
    "stm32/Drivers/BSP/${RUNTIME_LIBRARY}_Nucleo/*.c"
    "stm32/Core/Inc/*.h"
    "stm32/Core/Src/*.h"
    "stm32/Core/Src/*.c"
    "stm32/*.s"
)
list(FILTER RT_SRCS EXCLUDE REGEX ".*_template.*")

add_library(rt STATIC ${RT_SRCS})
target_compile_definitions(rt PUBLIC ${RT_DEFINITIONS})
target_include_directories(rt PUBLIC ${RT_INCLUDE_DIRS})
target_link_libraries(rt PUBLIC freertos_kernel)

# core Library

file(GLOB_RECURSE CORE_SRCS CONFIGURE_DEPENDS
    "core/*.h"
    "core/*.cpp"
)
file(GLOB_RECURSE CORE_MODULES CONFIGURE_DEPENDS
    "core/*.cppm"
)

add_library(core STATIC ${CORE_SRCS})
target_sources(core PUBLIC FILE_SET CXX_MODULES FILES ${CORE_MODULES})
target_include_directories(core PUBLIC "core" "/")
target_link_libraries(core PUBLIC rt freertos_kernel)

# drivers Library

file(GLOB_RECURSE DRIVERS_SRCS CONFIGURE_DEPENDS
    "drivers/*.h"
    "drivers/*.cpp"
)
file(GLOB_RECURSE DRIVERS_MODULES CONFIGURE_DEPENDS
    "drivers/*.cppm"
)

add_library(drivers STATIC ${DRIVERS_SRCS})
target_sources(drivers PUBLIC FILE_SET CXX_MODULES FILES ${DRIVERS_MODULES})
target_include_directories(drivers PUBLIC "drivers")
target_link_libraries(drivers PUBLIC core)

# Examples

file(GLOB EXAMPLES CONFIGURE_DEPENDS "examples/*.cpp")

foreach(EXAMPLE_FILE ${EXAMPLES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)

    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
    target_link_libraries(${EXAMPLE_NAME} PUBLIC drivers)

    add_custom_target(${EXAMPLE_NAME}-upload
        COMMAND STM32_Programmer_CLI --connect port=swd --download "$<TARGET_FILE:${EXAMPLE_NAME}>" -hardRst || exit 0 # Ignore error if no device is connected.
        DEPENDS ${EXAMPLE_NAME}
    )
endforeach()
