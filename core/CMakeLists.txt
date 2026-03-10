file(GLOB_RECURSE CORE_HEADERS CONFIGURE_DEPENDS "*.h")
list(FILTER CORE_HEADERS EXCLUDE REGEX "(rt_threading\..*|FreeRTOSConfig\.h)$")
file(GLOB_RECURSE CORE_SRCS CONFIGURE_DEPENDS "*.cpp")
list(FILTER CORE_SRCS EXCLUDE REGEX "rt_threading\..*$")

add_library(atmc.Core STATIC ${CORE_SRCS})
target_include_directories(atmc.Core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(atmc.Core PUBLIC rt sys sys.Containers sys.Threading sys.BuildSupport.CompilerOptions sys.BuildSupport.WarningsAsErrors)

target_lint_clang_tidy(atmc.Core "-header-filter=\"^${CMAKE_CURRENT_SOURCE_DIR}/(?!FreeRTOSConfig\\.h).*\"" ${CORE_HEADERS})
