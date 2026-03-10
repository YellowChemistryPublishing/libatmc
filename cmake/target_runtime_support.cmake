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
