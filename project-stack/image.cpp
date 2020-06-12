#include "image.hpp"

#include "common/logging.hpp"

#include <opencv2/calib3d.hpp>
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
 * @brief Detect features for alignment
 *
 * @param feature2D algorithm for detecting features
 */
void Image::detectFeatures(const cv::Ptr<cv::Feature2D>& feature2D) {
  cv::Mat imageGrey;
  cv::cvtColor(image, imageGrey, cv::COLOR_BGR2GRAY);

  // Scale to a max width and height of 1000 px
  // TODO(WattsUp): fix transformation from scaled version to full size
  // double ratio =
  //     static_cast<double>(imageGrey.cols) /
  //     static_cast<double>(imageGrey.rows);
  // if (ratio > 1.0) {
  //   cv::resize(imageGrey, imageGrey,
  //              cv::Size(1000, static_cast<int>(1000 / ratio)));
  // } else {
  //   cv::resize(imageGrey, imageGrey,
  //              cv::Size(static_cast<int>(1000 * ratio), 1000));
  // }

  feature2D->detectAndCompute(imageGrey, cv::noArray(), keyPoints, descriptors);
  cv::drawKeypoints(image, keyPoints, image, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

/**
 * @brief Evaluate number of matches between image and a reference image given
 * their detected features and generate a list of corresponding matches
 *
 * @param matcher algorithm for matching features
 * @param reference image
 * @return size_t number of good matches used to align
 */
size_t Image::generateAlignment(const cv::Ptr<cv::DescriptorMatcher>& matcher,
                                const Image& reference) {
  // Find the best two matches for each descriptor
  std::vector<std::vector<cv::DMatch>> matches;
  matcher->knnMatch(descriptors, reference.descriptors, matches, 2);

  std::vector<cv::DMatch> filteredMatches;
  pointsImage.clear();
  pointsReference.clear();

  // Ratio test, identify strong key points that have a first match that is
  // closer than 75% of its second match. Collect the corresponding key points
  // that are a good match
  static constexpr double ratioTest = 0.75;
  for (const std::vector<cv::DMatch>& matchPair : matches) {
    if (matchPair[0].distance < ratioTest * matchPair[1].distance) {
      filteredMatches.emplace_back(matchPair[0]);
      pointsImage.emplace_back(keyPoints[matchPair[0].queryIdx].pt);
      pointsReference.emplace_back(
          reference.keyPoints[matchPair[0].trainIdx].pt);
    }
  }

  return filteredMatches.size();
}

/**
 * @brief Apply alignment generated in generateAlignment
 *
 */
void Image::applyAlignment() {
  if (pointsImage.empty()) {
    throw std::logic_error(
        "generateAlignment must be called before applyAlignment");
  }
  // cv::drawMatches(image, keyPoints, reference.image, reference.keyPoints,
  //                 filteredMatches, imageGrey);

  // // Find homography transformation between the key points
  // cv::Mat homography =
  //     cv::findHomography(pointsImage, pointsReference, cv::RANSAC);
  // cv::warpPerspective(image, imageGrey, homography, image.size());

  // Find affine transformation between the key points
  // Assumes the perspective does not change
  cv::Mat affine = cv::estimateAffine2D(pointsImage, pointsReference);
  cv::warpAffine(image, image, affine, image.size());
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

/**
 * @brief Compare two matches, sorts smallest to largest distance
 *
 * @param i match 1
 * @param j match 2
 * @return i.distance < j.distance
 */
bool Image::compareMatches(const cv::DMatch& i, const cv::DMatch& j) {
  return i.distance < j.distance;
}

}  // namespace pancake