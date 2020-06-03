#include "logging.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <vector>

namespace common {
namespace logging {

spdlog::sink_ptr sinkFile    = nullptr;
spdlog::sink_ptr sinkConsole = nullptr;

/**
 * @brief Configure logging with a file, optional console, or neither
 *
 * @param filename of the log file, nullptr for no logging to file. Configures
 * level to debug and higher or info if release build
 * @param useConsole will use a console output window if true. Configures level
 * to warning and higher
 */
void configure(const char* filename, bool useConsole) {
  sinkFile    = nullptr;
  sinkConsole = nullptr;
  std::vector<spdlog::sink_ptr> sinks;

  if (useConsole) {
#ifdef WIN32
#ifndef WIN_CONSOLE
    // Attempt to create a console
    if (AllocConsole() != 0) {
      HWND hwnd = GetConsoleWindow();
      if (hwnd != nullptr) {
        HMENU hMenu = GetSystemMenu(hwnd, FALSE);
        if (hMenu != nullptr) {
          DeleteMenu(hMenu, SC_CLOSE, MF_BYCOMMAND);
        }
      }
      // NOLINTNEXTLINE (cppcoreguidelines-pro-type-cstyle-cast)
      freopen_s((FILE**)stdout, "CONOUT$", "w", stdout);
      // NOLINTNEXTLINE (cppcoreguidelines-pro-type-cstyle-cast)
      freopen_s((FILE**)stderr, "CONOUT$", "w", stderr);
    } else {
      spdlog::error("Error allocing a console: {}", GetLastError());
      throw std::exception("Log console initialization failed");
    }
#endif /* WIN_CONSOLE */
#endif /* WIN32 */

    sinkConsole = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    sinkConsole->set_level(spdlog::level::warn);
    sinks.emplace_back(sinkConsole);
  }

  // Create a file sink
  if (filename != nullptr) {
    sinkFile = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        filename, MAX_FILE_SIZE, MAX_FILE_COUNT);
#ifdef DEBUG
    sinkFile->set_level(spdlog::level::debug);
#else  /* !DEBUG */
    sinkFile->set_level(spdlog::level::info);
#endif /* DEBUG */
    sinks.emplace_back(sinkFile);
  }

  std::shared_ptr<spdlog::logger> logger =
      std::make_shared<spdlog::logger>("", begin(sinks), end(sinks));
  spdlog::set_default_logger(logger);

  spdlog::set_pattern("[%P:%t:%m%d/%H:%M:%S.%e][%^%=7l%$] %v");

  // Set logger level to lowest so the sinks decide which level to log
  spdlog::set_level(spdlog::level::trace);
}
}  // namespace logging
}  // namespace common