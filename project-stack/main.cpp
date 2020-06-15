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

    for (const boost::filesystem::path& file : arguments.getFiles()) {
      images.emplace_back(pancake::Image(file));
      images.back().detectFeatures(feature2D);
    }

    cv::Ptr<cv::DescriptorMatcher> matcher;
    matcher = cv::BFMatcher::create();

    // Use each image as a reference to find the best one to use
    // Best: highest minimum number of matches, i.e. reference that matches with
    // all of them the best rather than a couple really good
    auto refItr                              = images.begin();
    size_t maxMinGoodMatches                 = 0;
    const pancake::Image* bestReferenceImage = &(*refItr);
    for (; refItr != images.end(); ++refItr) {
      size_t minGoodMatches = std::numeric_limits<size_t>::max();
      auto itr              = refItr;
      ++itr;
      while (itr != refItr) {
        if (itr != images.end()) {
          size_t goodMatches = (*itr).generateAlignment(matcher, *refItr);
          minGoodMatches     = MIN(minGoodMatches, goodMatches);
          ++itr;
        } else {
          itr = images.begin();
        }
      }
      spdlog::debug("Reference had {} minimum good matches", minGoodMatches);
      if (minGoodMatches > maxMinGoodMatches) {
        maxMinGoodMatches  = minGoodMatches;
        bestReferenceImage = &(*refItr);
      }
    }

    // Use that best reference image to warp transform the rest
    std::vector<double> cropRect(4);
    cropRect[0] = 0;                                   // Top
    cropRect[1] = std::numeric_limits<double>::max();  // Right
    cropRect[2] = std::numeric_limits<double>::max();  // Bottom
    cropRect[3] = 0;                                   // Left
    for (pancake::Image& image : images) {
      image.generateAlignment(matcher, *bestReferenceImage);
      std::vector<double> rect = image.applyAlignment();

      cropRect[0] = MAX(cropRect[0], rect[0]);  // Top
      cropRect[1] = MIN(cropRect[1], rect[1]);  // Right
      cropRect[2] = MIN(cropRect[2], rect[2]);  // Bottom
      cropRect[3] = MAX(cropRect[3], rect[3]);  // Left
    }

    // Compute the gradient of each image and normalize per pixel
    cv::Mat maxGradient;
    for (pancake::Image& image : images) {
      spdlog::debug("Cropping image");
      image.crop(cropRect);
      spdlog::debug("Computing gradient");
      image.computeGradient();
      if (maxGradient.empty()) {
        image.computeGradient().copyTo(maxGradient);
      } else {
        maxGradient = cv::max(maxGradient, image.computeGradient());
      }
    }
    for (pancake::Image& image : images) {
      image.normalizeGradient(maxGradient);
    }

    cv::Mat outputImage =
        cv::Mat::zeros(images.front().size(), images.front().type());
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

    if (arguments.outputGradients()) {
      int i = 0;
      for (const pancake::Image& image : images) {
        auto path = boost::filesystem::change_extension(
            arguments.getOutput(), std::to_string(i) + ".jpg");
        image.save(path, true);
        ++i;
      }
    }
  } catch (const std::exception& e) {
    spdlog::error(e.what());
    return 1;
  }

  return 0;
}