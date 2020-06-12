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
  // cv::drawKeypoints(image, keyPoints, image, cv::Scalar::all(-1),
  //                   cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
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
 * @return std::vector<double> minimum rectangle for cropping purposes: top,
 * right, left, bottom
 */
std::vector<double> Image::applyAlignment() {
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

  // Get bounding box
  std::vector<cv::Point2d> corners(4);
  corners[0] = cv::Point2d(0, 0);
  corners[1] = cv::Point2d(static_cast<double>(image.cols), 0);
  corners[2] = cv::Point2d(static_cast<double>(image.cols),
                           static_cast<double>(image.rows));
  corners[3] = cv::Point2d(0, static_cast<double>(image.rows));
  for (cv::Point2d& point : corners) {
    cv::Point2d original = point;

    point.x = original.x * affine.at<double>(0, 0) +
              original.y * affine.at<double>(0, 1) + affine.at<double>(0, 2);
    point.y = original.x * affine.at<double>(1, 0) +
              original.y * affine.at<double>(1, 1) + affine.at<double>(1, 2);
  }

  // Find smallest rectangle that fits in the transformed image
  std::vector<double> rect(4);
  rect[0] = MAX(corners[0].y, corners[1].y);  // Top
  rect[1] = MIN(corners[1].x, corners[2].x);  // Right
  rect[2] = MIN(corners[2].y, corners[3].y);  // Bottom
  rect[3] = MAX(corners[0].x, corners[3].x);  // Left

  // corners[0]    = cv::Point2d(rect[3], rect[0]);
  // corners[1]    = cv::Point2d(rect[1], rect[0]);
  // corners[2]    = cv::Point2d(rect[1], rect[2]);
  // corners[3]    = cv::Point2d(rect[3], rect[2]);
  // cv::line(image, corners[0], corners[1], cv::Scalar(0, 255, 0));
  // cv::line(image, corners[1], corners[2], cv::Scalar(0, 255, 0));
  // cv::line(image, corners[2], corners[3], cv::Scalar(0, 255, 0));
  // cv::line(image, corners[3], corners[0], cv::Scalar(0, 255, 0));
  return rect;
}

/**
 * @brief Crop image to rectangle
 *
 * @param rect top, right, bottom, left
 */
void Image::crop(const std::vector<double>& rect) {
  int top    = static_cast<int>(ceil(rect[0]));
  int right  = static_cast<int>(floor(rect[1]));
  int bottom = static_cast<int>(floor(rect[2]));
  int left   = static_cast<int>(ceil(rect[3]));
  cv::Rect cropROI(left, top, right - left, bottom - top);
  image = image(cropROI);
}

/**
 * @brief Compute the gradient of the image by bluring then taking the laplacian
 *
 * @param double maximum value of the computed gradient
 */
double Image::computeGradient() {
  static constexpr int laplacianSize = 5;
  cv::Mat grey;
  cv::GaussianBlur(image, grey, cv::Size(laplacianSize, laplacianSize), 0);
  cv::cvtColor(grey, grey, cv::COLOR_BGR2GRAY);
  cv::Laplacian(grey, gradient, CV_64F, laplacianSize);
  gradient = cv::abs(gradient);

  static constexpr double blurSize = 2.5;
  cv::GaussianBlur(gradient, gradient, cv::Size(0, 0), blurSize);
  // cv::normalize(gradient, gradient, 0.0, 1.0, cv::NORM_MINMAX);
  double min;
  double max;
  cv::minMaxLoc(gradient, &min, &max);
  return max;
}

/**
 * @brief Quantize the gradient to integers for binary comparison
 *
 * @param maxGradientVal to scale to
 * @return const cv::Mat&
 */
const cv::Mat& Image::quantizeGradient(const double maxGradientVal) {
  cv::Mat tmp;
  gradient.convertTo(tmp, CV_32S,
                     std::numeric_limits<int32_t>::max() / maxGradientVal);
  gradient = tmp;
  return gradient;
}

/**
 * @brief Mask the image where its gradient != maxGradient
 *
 * @param dst destination image to add to
 * @param maxGradient to compare its gradient to
 */
void Image::combineByMaskGradient(cv::Mat& dst, const cv::Mat& maxGradient) {
  cv::compare(gradient, maxGradient, gradient, cv::CMP_EQ);
  cv::add(image, dst, dst, gradient);
}

/**
 * @brief Mask the image where its gradient != maxGradient
 *
 * @param dst destination image to add to
 * @param maxGradient to compare its gradient to
 * @param hue to color mask as
 */
void Image::combineByMaskGradient(cv::Mat& dst,
                                  const cv::Mat& maxGradient,
                                  const double hue) {
  image = cv::Scalar(hue, std::numeric_limits<uint8_t>::max(),
                     std::numeric_limits<uint8_t>::max());
  cv::cvtColor(image, image, cv::COLOR_HSV2BGR);
  cv::compare(gradient, maxGradient, gradient, cv::CMP_EQ);
  cv::add(image, dst, dst, gradient);
}

/**
 * @brief Save the image to a path
 *
 * @param path to save image to
 */
void Image::save(const boost::filesystem::path& path) const {
  spdlog::info("Saving image to {}", path);
  if (!cv::imwrite(path.string(), image)) {
    throw std::exception(("Failed to save image to " + path.string()).c_str());
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