set ( LIBRARY_INCLUDES
  "miniz"
)

set(gtest_force_shared_crt ON CACHE BOOL "Always use msvcrt.dll" FORCE)
add_subdirectory("googletest")
target_include_directories("test" SYSTEM PRIVATE "googletest/googletest/include")

target_sources("installer-win" PRIVATE "miniz/miniz.c")


find_package(spdlog CONFIG REQUIRED)

target_link_libraries("test" PRIVATE gtest)

foreach(target ${TARGETS})
  target_include_directories(${target} SYSTEM PRIVATE ${LIBRARY_INCLUDES})
  target_link_libraries(${target} PRIVATE spdlog::spdlog spdlog::spdlog_header_only)
endforeach()