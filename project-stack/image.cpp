#include "image.hpp"

#include "common/logging.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace pancake {

/**
 * @brief Construct a new Image:: Image object
 *
 * @param path to load image from
 */
Image::Image(const boost::filesystem::path& path) {
  spdlog::info("Loading image from {}", path);
  image = cv::imread(path.string(), cv::IMREAD_COLOR);
  if (image.data == nullptr) {
    throw std::exception(
        ("Failed to read iamge from " + path.string()).c_str());
  }
}

/**
 * @brief Save the image to a path
 *
 * @param path to save image to
 */
void Image::save(const boost::filesystem::path& path) const {
  spdlog::info("Saving image to {}", path);
  if (!cv::imwrite(path.string(), image)) {
    throw std::exception(("Failed to save iamge to " + path.string()).c_str());
  }
}

}  // namespace pancake