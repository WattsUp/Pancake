set (SRCS
  "main.cpp"
)

set (SRCS_UNIX
  "main_unix.cpp"
)

set (SRCS_WIN
  "main_win.cpp"
)

target_sources("TARGET" PRIVATE ${SRCS} $<IF:$<BOOL:${WIN32}>, ${SRCS_WIN}, ${SRCS_UNIX}>)

set (TEST_SRCS
)

target_sources("test" PRIVATE ${TEST_SRCS})