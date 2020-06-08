#include "common/logging.hpp"
#include "common/version.h"

#include <OpenImageIO/imageio.h>

#include <cstdio>
#include <cxxopts.hpp>
#include <exception>

/**
 * @brief Main entry point for program
 *
 * @param argc count of arguments
 * @param argv array of arguments
 * @return int zero on success, non-zero on failure
 */
int main(int argc, char* argv[]) {
  try {
    cxxopts::Options options("Pancake", "A photography stacking tool");
    options.positional_help("<files>");

    cxxopts::OptionAdder optionsAdder = options.add_options();
    optionsAdder("v,verbose", "Verbose output",
                 cxxopts::value<bool>()->default_value("false"));
    optionsAdder("d,dir", "Install directory",
                 cxxopts::value<std::string>()->default_value("temp"));
    optionsAdder("version", "Software version",
                 cxxopts::value<bool>()->default_value("false"));
    optionsAdder("h,help", "Print usage");
    optionsAdder("f,file", "Files to stack",
                 cxxopts::value<std::vector<std::string>>());

    options.parse_positional("file");
    cxxopts::ParseResult result = options.parse(argc, argv);

    if (result.count("help") != 0) {
      std::cout << options.help() << std::endl;
      return 0;
    }

    if (result.count("version") != 0) {
#if DEBUG
      std::cout << VERSION_STRING_FULL << std::endl;
#else  /* DEBUG */
      std::cout << VERSION_STRING << std::endl;
#endif /* DEBUG */
      return 0;
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

    if (result.count("file") == 0) {
      std::cout << "Required at least two files" << std::endl;
      return 1;
    }
    for (const std::string& file :
         result["file"].as<std::vector<std::string>>()) {
      spdlog::info("Positional argument: {}", file);
    }

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  const char* filename = "temp/foo.jpg";
  const int xres = 640, yres = 480;
  const int channels = 3;  // RGB
  unsigned char pixels[xres * yres * channels];

  std::unique_ptr<OIIO::ImageOutput> out = OIIO::ImageOutput::create(filename);
  if (!out) {
    return 1;
  }
  OIIO::ImageSpec spec(xres, yres, channels, OIIO::TypeDesc::UINT8);
  out->open(filename, spec);
  out->write_image(OIIO::TypeDesc::UINT8, static_cast<void*>(pixels));
  out->close();
  return 0;
}