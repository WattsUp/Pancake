set (SRCS
  "main.cpp"
  "arguments.cpp"
)

set (SRCS_UNIX
)

set (SRCS_WIN
)

target_sources("stack" PRIVATE ${SRCS} $<IF:$<BOOL:${WIN32}>, ${SRCS_WIN}, ${SRCS_UNIX}>)

set (TEST_SRCS
)

target_sources("test" PRIVATE ${TEST_SRCS})