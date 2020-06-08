#include "common/logging.hpp"
#include "common/version.h"

#include <OpenImageIO/imageio.h>

#include <cstdio>
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
#if DEBUG
    spdlog::info(VERSION_STRING_FULL);
#else  /* DEBUG */
    spdlog::info(VERSION_STRING);
#endif /* DEBUG */

    for (int i = 0; i < argc; ++i) {
      // NOLINTNEXTLINE (cppcoreguidelines-pro-bounds-pointer-arithmetic)
      spdlog::info("Argument: {}", argv[i]);
    }
  } catch (std::exception& e) {
    // Catch exceptions from spdlog
    puts(e.what());
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