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
    file(APPEND "${CMAKE_BINARY_DIR}/runtime_headers.h" "\n#ifdef __cplusplus\n}\n\n#undef register\n#endif\n")

    set(RT_INCLUDE_DIRS
        "${CMAKE_BINARY_DIR}"
        stm32
        stm32/Drivers/CMSIS/Include
        "stm32/Drivers/CMSIS/Device/ST/${RUNTIME_LIBRARY}/Include"
        "stm32/Drivers/${RUNTIME_LIBRARY}_HAL_Driver/Inc"
        stm32/Core/Src
        stm32/Core/Inc
        freertos/include
        core
    )
    set(RT_DEFINITIONS "${MCU_ID}=1" "${RUNTIME_LIBRARY}=1")
    set(RT_DEFINITIONS ${RT_DEFINITIONS} "STM32=1" "HAL_ADC_MODULE_ENABLED")
elseif(BOARD_NAME STREQUAL "Hosted")
    file(WRITE "${CMAKE_BINARY_DIR}/runtime_headers.h" "#pragma once\n")

    set(RT_INCLUDE_DIRS
        "${CMAKE_BINARY_DIR}"
        generic
    )
    set(RT_DEFINITIONS "HOSTED=1")
endif()
