#include "arguments.hpp"
#include "common/logging.hpp"
#include "image.hpp"

#include <exception>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

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
  } catch (const std::exception& e) {
    spdlog::error(e.what());
    arguments.helpAndExit();
    return 1;
  }

  try {
    cv::Ptr<cv::Feature2D> feature2D;
    // feature2D = cv::ORB::create(1000);
    feature2D = cv::xfeatures2d::SIFT::create();

    bool affineLoaded = true;
    for (const boost::filesystem::path& file : arguments.getFiles()) {
      images.emplace_back(pancake::Image(file));
      affineLoaded = images.back().detectFeatures(feature2D) && affineLoaded;
    }

    // If not all the images have an affine transformation loaded from file,
    // rematch all
    if (!affineLoaded) {
      cv::Ptr<cv::DescriptorMatcher> matcher;
      matcher = cv::BFMatcher::create();

      // Align each image to each remaining image, keeping the reference image
      // that yields the most number of matches
      auto image1 = images.begin();
      for (; image1 != images.end(); ++image1) {
        spdlog::debug("Aligning {}", (*image1).getName());
        auto image2 = image1;
        ++image2;
        for (; image2 != images.end(); ++image2) {
          (*image1).generateBestAlignment(matcher, *image2);
        }
      }
    }

    // Apply alignment and compute largest rectangle that contains all images,
    // crop to that rectangle
    std::vector<double> cropRect(4);
    cropRect[0] = 0;                                   // Top
    cropRect[1] = std::numeric_limits<double>::max();  // Right
    cropRect[2] = std::numeric_limits<double>::max();  // Bottom
    cropRect[3] = 0;                                   // Left
    for (pancake::Image& image : images) {
      std::vector<double> rect = image.applyAlignment();

      cropRect[0] = MAX(cropRect[0], rect[0]);  // Top
      cropRect[1] = MIN(cropRect[1], rect[1]);  // Right
      cropRect[2] = MIN(cropRect[2], rect[2]);  // Bottom
      cropRect[3] = MAX(cropRect[3], rect[3]);  // Left
    }

    // Compute the gradient of each image and normalize per pixel
    cv::Mat maxGradient;
    int i = 0;
    for (pancake::Image& image : images) {
      spdlog::debug("Cropping image");
      image.crop(cropRect);
      if (arguments.outputAlignments()) {
        auto path = boost::filesystem::change_extension(
            arguments.getOutput(), std::to_string(i) + ".aligned.jpg");
        ++i;
        image.save(path);
      }

      spdlog::debug("Computing gradient");
      image.computeGradient();
      if (maxGradient.empty()) {
        image.computeGradient().copyTo(maxGradient);
      } else {
        maxGradient = cv::max(maxGradient, image.computeGradient());
      }
    }
    // Normalize the gradient per pixel: largest gradient is 1.0 and the others
    // are proportionate to that maximum
    for (pancake::Image& image : images) {
      image.normalizeGradient(maxGradient);
    }

    if (arguments.outputGradients()) {
      i = 0;
      for (const pancake::Image& image : images) {
        auto path = boost::filesystem::change_extension(
            arguments.getOutput(), std::to_string(i) + "gradient.jpg");
        image.save(path, true);
        ++i;
      }
    }

    cv::Mat outputImage =
        cv::Mat::zeros(images.front().size(), images.front().type());
    spdlog::debug("Combining output image");
    pancake::Image::combine(outputImage, images);

    if (!cv::imwrite(arguments.getOutput().string(), outputImage)) {
      throw std::exception(
          ("Failed to save image to " + arguments.getOutput().string())
              .c_str());
    }

    if (arguments.outputDepthMap()) {
      double hue     = 0.0;
      double hueStep = 180.0 / images.size();  // NOLINT
      spdlog::debug("Creating depth map");
      for (pancake::Image& image : images) {
        image.colorize(hue);
        hue += hueStep;
      }
      pancake::Image::combine(outputImage, images);
      auto path = boost::filesystem::change_extension(arguments.getOutput(),
                                                      ".depth.jpg");
      if (!cv::imwrite(path.string(), outputImage)) {
        throw std::exception(
            ("Failed to save image to " + arguments.getOutput().string())
                .c_str());
      }
    }
  } catch (const std::exception& e) {
    spdlog::error(e.what());
    return 1;
  }

  return 0;
}