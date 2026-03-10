if(LIBATMC_EXAMPLES)
    file(GLOB EXAMPLES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp")
    foreach(EXAMPLE_FILE ${EXAMPLES})
        get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
        set(EXAMPLE_NAME "example_${EXAMPLE_NAME}")

        add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
        target_link_libraries(${EXAMPLE_NAME} PUBLIC atmc.Drivers sys.BuildSupport.CompilerOptions sys.BuildSupport.WarningsAsErrors)

        target_lint_clang_tidy(${EXAMPLE_NAME} "-header-filter=\"^${CMAKE_CURRENT_SOURCE_DIR}/.*\"" ${EXAMPLE_FILE})
        target_run_stm32_programmer(${EXAMPLE_NAME})
    endforeach()
endif()
