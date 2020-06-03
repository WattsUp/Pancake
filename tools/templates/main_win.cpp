#include "common/logging.hpp"
#include "common/version.h"

#include <Windows.h>

#include <cstdio>
#include <exception>

/**
 * @brief Main entry point for program
 *
 * @param lpCmdLine command line string
 * @return int zero on success, non-zero on failure
 */
int WINAPI WinMain(HINSTANCE /* hInstance */,
                   HINSTANCE /* hPrevInstance */,
                   char* lpCmdline,
                   int /* nShowCmd */) {
  try {
    common::logging::configure("log.log", true);
  } catch (const std::exception& e) {
    puts(e.what());
  }
#if DEBUG
  spdlog::info(VERSION_STRING_FULL);
#else  /* DEBUG */
  spdlog::info(VERSION_STRING);
#endif /* DEBUG */

  spdlog::info("Argument: {}", lpCmdline);
  return 0;
}
