set (RESOURCE_DIRS
  "include"
)

if (WIN32)
  install(DIRECTORY ${RESOURCE_DIRS}
          DESTINATION .)
  install(DIRECTORY . DESTINATION .
          FILES_MATCHING PATTERN "*.dll")
endif()