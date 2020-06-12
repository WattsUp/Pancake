#include "arguments.hpp"
#include "common/logging.hpp"
#include "image.hpp"

#include <exception>
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
    for (pancake::Image& image : images) {
      image.generateAlignment(matcher, *bestReferenceImage);
      image.applyAlignment();
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