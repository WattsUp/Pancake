#include "arguments.hpp"

#include "common/logging.hpp"
#include "common/version.h"

#include <cxxopts.hpp>

namespace pancake {

/**
 * @brief Parse arguments from the command line argument array
 *
 * @param argc count of arguments
 * @param argv array of arguments
 */
void Arguments::parse(
    int argc,
    char* argv[]) {  // NOLINT (cppcoreguidelines-avoid-c-arrays)
  cxxopts::Options options("Pancake", "A photography stacking tool");
  options.positional_help("<paths...>");

  cxxopts::OptionAdder optionsAdder = options.add_options();
  optionsAdder("v,verbose", "Verbose output",
               cxxopts::value<bool>()->default_value("false"));
  optionsAdder("version", "Software version",
               cxxopts::value<bool>()->default_value("false"));
  optionsAdder("r,recursive", "Search <files> recursively",
               cxxopts::value<bool>()->default_value("false"));
  optionsAdder("h,help", "Print usage");
  optionsAdder("p,path", "Files (or directories) to stack",
               cxxopts::value<std::vector<std::string>>());

  options.parse_positional("path");
  cxxopts::ParseResult result = options.parse(argc, argv);

  if (result.count("help") != 0) {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  if (result.count("version") != 0) {
#if DEBUG
    std::cout << VERSION_STRING_FULL << std::endl;
#else  /* DEBUG */
    std::cout << VERSION_STRING << std::endl;
#endif /* DEBUG */
    exit(0);
  }

#if DEBUG
  spdlog::set_level(spdlog::level::info);
#else  /* DEBUG */
  spdlog::set_level(spdlog::level::warn);
#endif /* DEBUG */
  if (result.count("verbose") != 0 && result["verbose"].as<bool>()) {
#if DEBUG
    spdlog::set_level(spdlog::level::debug);
#else  /* DEBUG */
    spdlog::set_level(spdlog::level::info);
#endif /* DEBUG */
  }

  if (result.count("path") == 0) {
    throw std::invalid_argument("Requires at least two files");
  }

  for (const std::string& path :
       result["path"].as<std::vector<std::string>>()) {
    spdlog::info("Path: {}", path);
  }
}

}  // namespace pancake