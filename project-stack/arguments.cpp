#include "arguments.hpp"

#include "common/logging.hpp"
#include "common/version.h"

namespace pancake {

/**
 * @brief Construct a new Arguments:: Arguments object
 *
 */
Arguments::Arguments() : options("Pancake", "A photography stacking tool") {
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
  optionsAdder("o,output", "Output file location",
               cxxopts::value<std::string>());
  optionsAdder("depth-map",
               "Output image representing a depth map of the scene (images "
               "must be in order)",
               cxxopts::value<bool>()->default_value("false"));
  optionsAdder("output-gradients",
               "Output image representing the gradient of each image",
               cxxopts::value<bool>()->default_value("false"));
  optionsAdder("output-alignments", "Output each aligned image",
               cxxopts::value<bool>()->default_value("false"));
}

/**
 * @brief Parse arguments from the command line argument array
 *
 * @param argc count of arguments
 * @param argv array of arguments
 */
void Arguments::parse(
    int argc,
    char* argv[]) {  // NOLINT (cppcoreguidelines-avoid-c-arrays)
  options.parse_positional("path");
  cxxopts::ParseResult result = options.parse(argc, argv);

  if (result.count("help") != 0) {
    helpAndExit();
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
  spdlog::set_level(spdlog::level::debug);
#else  /* DEBUG */
  spdlog::set_level(spdlog::level::info);
#endif /* DEBUG */
  if (result.count("verbose") != 0) {
    spdlog::set_level(spdlog::level::debug);
  }

  if (result.count("path") == 0) {
    throw std::invalid_argument("Requires at least two files");
  }

  if (result.count("output") == 0) {
    throw std::invalid_argument("Requires an output file");
  }
  output = boost::filesystem::path(result["output"].as<std::string>());

  bool recursive = (result.count("recursive") != 0);

  static const boost::regex extFilter(
      "\\.(bmp|dip|jpeg|jpg|jpe|jp2|png|webp|pbm|pgm|ppm|pxm|pnm|pfm|sr|ras|"
      "tiff|tif|exr|hdr|pic)",
      boost::regex::icase);
  for (const std::string& pathStr :
       result["path"].as<std::vector<std::string>>()) {
    boost::filesystem::path path(pathStr);
    addPath(path, extFilter, recursive);
  }
  if (files.size() < 2) {
    throw std::invalid_argument("Requires at least two files");
  }

  if (result.count("depth-map") != 0) {
    depthMap = true;
  }

  if (result.count("output-gradients") != 0) {
    gradients = true;
  }

  if (result.count("output-alignments") != 0) {
    alignments = true;
  }
}

/**
 * @brief Print help message
 *
 */
void Arguments::helpAndExit() {
  std::cout << options.help() << std::endl;
  exit(0);
}

/**
 * @brief If path is a compatible file, add to files. If directory, search its
 * children
 *
 * @param path to add
 * @param extFilter regex to match extension to
 * @param recursive true will search directories of directories, false will not
 */
void Arguments::addPath(const boost::filesystem::path& path,
                        const boost::regex& extFilter,
                        bool recursive) {
  if (!boost::filesystem::exists(path)) {
    spdlog::warn("Path {} does not exist", path);
    return;
  }
  if (boost::filesystem::is_regular_file(path)) {
    if (path.extension() == ".yml") {
      spdlog::debug("Skipping {}, likely features file", path);
      return;
    }
    if (!boost::regex_match(path.extension().string(), extFilter)) {
      spdlog::warn("Path {} does not have a compatible extension", path);
      return;
    }
    files.emplace_back(path);
  } else if (boost::filesystem::is_directory(path)) {
    boost::filesystem::directory_iterator itr(path);
    boost::filesystem::directory_iterator end;
    for (; itr != end; ++itr) {
      if (boost::filesystem::is_directory(*itr) && !recursive) {
        continue;
      }
      addPath(*itr, extFilter, recursive);
    }
  } else {
    spdlog::warn("Path {} is an unknown type", path);
    return;
  }
}

}  // namespace pancake