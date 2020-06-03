set (RESOURCE_DIRS
  "include"
)

if (WIN32)
  install(DIRECTORY ${RESOURCE_DIRS}
          DESTINATION .)
endif()