cmake_minimum_required(VERSION 3.30)

include(cmake/stm32_gcc.cmake)

project(libatmc)
enable_language(ASM C CXX)

set(BOARD_NAME "STM32H753ZI")

# Helpers.

find_program(RUN_CPPCHECK cppcheck PATHS "C:/Program Files/Cppcheck" "/usr/bin")
if (NOT RUN_CPPCHECK)
    message(FATAL_ERROR "Cppcheck not found. Required for static analysis.")
endif()
set(LINT_CPPCHECK ${RUN_CPPCHECK} --suppress=preprocessorErrorDirective --enable=all --inconclusive --inline-suppr --force --max-configs=64 -DconfigUSE_16_BIT_TICKS)

# Give up for now, very inconvenient setup for arm-none-eabi-gcc.
# find_program(RUN_CLANG_TIDY clang-tidy PATHS "C:/Program Files/LLVM/bin" "/usr/bin")
# if (NOT RUN_CLANG_TIDY)
#     message(FATAL_ERROR "clang-tidy not found. Required for static analysis.")
# endif()
# set(LINT_CLANG_TIDY ${RUN_CLANG_TIDY} -checks=*)

find_program(RUN_STM32_PROGRAMMER_CLI STM32_Programmer_CLI PATHS "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin" "/usr/bin")
if (NOT RUN_STM32_PROGRAMMER_CLI)
    message(WARNING "STM32_Programmer_CLI not found. Flashing will be disabled.")
else()
    function(create_stm32_programmer_target EXAMPLE_NAME)
        add_custom_target(upload_${EXAMPLE_NAME}
            COMMAND ${RUN_STM32_PROGRAMMER_CLI} --connect port=swd --download "$<TARGET_FILE:${EXAMPLE_NAME}>" -hardRst || exit 0 # Ignore error if no device is connected.
            DEPENDS ${EXAMPLE_NAME}
        )
    endfunction()
endif()

# Compiler settings.

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 26 REQUIRED)
set(CMAKE_CXX_SCAN_FOR_MODULES FALSE)

if (NOT (CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU"))
    message(WARNING "Compiler set is not gcc, expect errors.")
endif()

# Target specific.

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
if (RUNTIME_LIBRARY STREQUAL "STM32H7xx")
    set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_CURRENT_LIST_DIR}/stm32/STM32H7xx_DMA.ld\"")
    set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -T \"${CMAKE_CURRENT_LIST_DIR}/stm32/STM32H7xx_DMA.ld\"")
endif()

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
    "${CMAKE_SOURCE_DIR}/stm32"
    "${CMAKE_SOURCE_DIR}/stm32/Drivers/CMSIS/Include"
    "${CMAKE_SOURCE_DIR}/stm32/Drivers/CMSIS/Device/ST/${RUNTIME_LIBRARY}/Include"
    "${CMAKE_SOURCE_DIR}/stm32/Drivers/${RUNTIME_LIBRARY}_HAL_Driver/Inc"
    "${CMAKE_SOURCE_DIR}/stm32/Core/Src"
    "${CMAKE_SOURCE_DIR}/stm32/Core/Inc"
    "${CMAKE_SOURCE_DIR}/freertos/include"
    "${CMAKE_SOURCE_DIR}/core"
)
set(RT_DEFINITIONS "${MCU_ID}=1" "${RUNTIME_LIBRARY}=1")
if (MCU_ID MATCHES "STM32")
    set(RT_DEFINITIONS ${RT_DEFINITIONS} "STM32=1" "HAL_ADC_MODULE_ENABLED")
endif()

# FreeRTOS

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE ${RT_INCLUDE_DIRS})
target_compile_definitions(freertos_config INTERFACE ${RT_DEFINITIONS} configUSE_PASSIVE_IDLE_HOOK=0 projCOVERAGE_TEST=0)

target_compile_definitions(freertos_config INTERFACE ${definitions})
target_compile_options(freertos_config INTERFACE ${options})

set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)
set(FREERTOS_PORT "GCC_ARM_CM7" CACHE STRING "" FORCE)

add_subdirectory(freertos)

# stm32 Board Support

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

# libcxxext

set(PLATFORM_SUPPORT_THREADING_HEADER "${CMAKE_SOURCE_DIR}/core/rt_threading_embedded.h")
set(PLATFORM_SUPPORT_THREADING_LIBRARY "${CMAKE_SOURCE_DIR}/core/rt_threading_embedded.cpp")
add_subdirectory(libcxxext)
target_include_directories(sys.Threading PUBLIC "/")
target_link_libraries(sys.Threading PUBLIC rt freertos_kernel)
set_target_properties(sys.Threading PROPERTIES
    C_CPPCHECK ${LINT_CPPCHECK}
    CXX_CPPCHECK ${LINT_CPPCHECK}
    C_CLANG_TIDY ${LINT_CLANG_TIDY}
    CXX_CLANG_TIDY ${LINT_CLANG_TIDY}
)

# core Library

file(GLOB_RECURSE CORE_SRCS CONFIGURE_DEPENDS
    "core/*.h"
    "core/*.cpp"
)
list(FILTER CORE_SRCS EXCLUDE REGEX "${CMAKE_SOURCE_DIR}/core/rt_threading_embedded.cpp")
file(GLOB_RECURSE CORE_MODULES CONFIGURE_DEPENDS
    "core/*.cppm"
)

add_library(core STATIC ${CORE_SRCS})
target_sources(core PUBLIC FILE_SET CXX_MODULES FILES ${CORE_MODULES})
target_include_directories(core PUBLIC "/" "core")
target_link_libraries(core PUBLIC rt freertos_kernel sys sys.Containers sys.Threading sys.BuildSupport.CompilerOptions sys.BuildSupport.WarningsAsErrors)
set_target_properties(core PROPERTIES
    C_CPPCHECK ${LINT_CPPCHECK}
    CXX_CPPCHECK ${LINT_CPPCHECK}
    C_CLANG_TIDY ${LINT_CLANG_TIDY}
    CXX_CLANG_TIDY ${LINT_CLANG_TIDY}
)

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
target_link_libraries(drivers PUBLIC core sys.BuildSupport.CompilerOptions sys.BuildSupport.WarningsAsErrors)
set_target_properties(drivers PROPERTIES
    C_CPPCHECK ${LINT_CPPCHECK}
    CXX_CPPCHECK ${LINT_CPPCHECK}
    C_CLANG_TIDY ${LINT_CLANG_TIDY}
    CXX_CLANG_TIDY ${LINT_CLANG_TIDY}
)

# Examples

file(GLOB EXAMPLES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/examples/*.cpp")

foreach(EXAMPLE_FILE ${EXAMPLES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
    set(EXAMPLE_NAME "example_${EXAMPLE_NAME}")

    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
    target_link_libraries(${EXAMPLE_NAME} PUBLIC drivers sys.BuildSupport.CompilerOptions sys.BuildSupport.WarningsAsErrors)
    set_target_properties(${EXAMPLE_NAME} PROPERTIES
        C_CPPCHECK ${LINT_CPPCHECK}
        CXX_CPPCHECK ${LINT_CPPCHECK}
        C_CLANG_TIDY ${LINT_CLANG_TIDY}
        CXX_CLANG_TIDY ${LINT_CLANG_TIDY}
    )

    if (RUN_STM32_PROGRAMMER_CLI)
        create_stm32_programmer_target(${EXAMPLE_NAME})
    endif()
endforeach()
