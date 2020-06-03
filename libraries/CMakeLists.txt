set ( LIBRARY_INCLUDES
  "spdlog/include"
  "miniz"
)

foreach(target ${TARGETS})
  target_include_directories(${target} SYSTEM PRIVATE ${LIBRARY_INCLUDES})
endforeach()

set(gtest_force_shared_crt ON CACHE BOOL "Always use msvcrt.dll" FORCE)
add_subdirectory("googletest")
target_include_directories("test" SYSTEM PRIVATE "googletest/googletest/include")

target_sources("installer-win" PRIVATE "miniz/miniz.c")
