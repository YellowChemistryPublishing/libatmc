find_program(RUN_STM32_PROGRAMMER_CLI STM32_Programmer_CLI PATHS "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin" "/usr/bin")

if(NOT RUN_STM32_PROGRAMMER_CLI)
    message(WARNING "STM32_Programmer_CLI not found. Flashing will be disabled.")
endif()

function(target_run_stm32_programmer TARGET_NAME)
    if(RUN_STM32_PROGRAMMER_CLI)
        add_custom_target(upload_${TARGET_NAME}
            COMMAND ${RUN_STM32_PROGRAMMER_CLI} --connect port=swd --download "$<TARGET_FILE:${TARGET_NAME}>" -hardRst || exit 0 # Ignore error if no device is connected.
            DEPENDS ${TARGET_NAME}
        )
    endif()
endfunction()
