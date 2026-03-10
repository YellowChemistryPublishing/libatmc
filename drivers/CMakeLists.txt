file(GLOB_RECURSE DRIVERS_HEADERS CONFIGURE_DEPENDS "*.h")
list(FILTER DRIVERS_HEADERS EXCLUDE REGEX "FreeRTOSConfig\.h$")

add_library(atmc.Drivers INTERFACE)
target_include_directories(atmc.Drivers INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(atmc.Drivers INTERFACE atmc.Core sys.BuildSupport.CompilerOptions sys.BuildSupport.WarningsAsErrors)

target_lint_clang_tidy(atmc.Drivers "-header-filter=\"^${CMAKE_CURRENT_SOURCE_DIR}/.*\"" ${DRIVERS_HEADERS})
