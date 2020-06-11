#include "arguments.hpp"
#include "common/logging.hpp"
#include "image.hpp"

#include <exception>

/**
 * @brief Main entry point for program
 *
 * @param argc count of arguments
 * @param argv array of arguments
 * @return int zero on success, non-zero on failure
 */
int main(int argc, char* argv[]) {
  pancake::Arguments arguments;

  std::list<pancake::Image> images;

  try {
    arguments.parse(argc, argv);

    for (const boost::filesystem::path& file : arguments.getFiles()) {
      images.emplace_back(pancake::Image(file));
    }

    int i = 0;
    for (const pancake::Image& image : images) {
      std::string path = "temp/" + std::to_string(i) + ".jpg";
      image.save(boost::filesystem::path(path));
      ++i;
    }

  } catch (const std::exception& e) {
    spdlog::error(e.what());
    return 1;
  }

  return 0;
}