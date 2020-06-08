set (SRCS
  "main.cpp"
  "main.rc"
)

set_property(SOURCE "main.rc" APPEND PROPERTY OBJECT_DEPENDS
  "installer.ico"
  "${CMAKE_SOURCE_DIR}/common/Version.h"
  "resources.h"
  "${CMAKE_SOURCE_DIR}/archive.zip"
)

add_dependencies("installer-win" "package")

target_sources("installer-win" PRIVATE ${SRCS})
