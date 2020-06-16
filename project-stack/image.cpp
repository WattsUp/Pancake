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
Image::Image(const boost::filesystem::path& path) : path(path) {
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
 * @return bool true if affine transformation was loaded from file, false
 * otherwise
 */
bool Image::detectFeatures(const cv::Ptr<cv::Feature2D>& feature2D) {
  cv::Mat imageGrey;
  cv::cvtColor(image, imageGrey, cv::COLOR_BGR2GRAY);
  boost::filesystem::path featuresPath(path.string() + ".yml");
  if (boost::filesystem::exists(featuresPath)) {
    spdlog::info("Loading features from {}", featuresPath);
    cv::FileStorage fs(featuresPath.string(), cv::FileStorage::READ);
    cv::read(fs["Key Points"], keyPoints);
    cv::read(fs["Descriptors"], descriptors);
    if (!fs["Affine"].empty()) {
      cv::read(fs["Affine"], totalAffine);
    }
    fs.release();
  }
  if (keyPoints.empty() || descriptors.empty()) {
    feature2D->detectAndCompute(imageGrey, cv::noArray(), keyPoints,
                                descriptors);
    spdlog::debug("Saving features to {}", featuresPath);
    cv::FileStorage fs(featuresPath.string(), cv::FileStorage::WRITE);
    cv::write(fs, "Key Points", keyPoints);
    cv::write(fs, "Descriptors", descriptors);
    fs.release();
  }

  // cv::drawKeypoints(image, keyPoints, image, cv::Scalar::all(-1),
  //                   cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  return !totalAffine.empty();
}

/**
 * @brief Evaluate number of matches between image and a reference image given
 * their detected features and generate a list of corresponding matches
 *
 * @param matcher algorithm for matching features
 * @param reference image
 */
void Image::generateBestAlignment(const cv::Ptr<cv::DescriptorMatcher>& matcher,
                                  const Image& reference) {
  // Find the best two matches for each descriptor
  std::vector<std::vector<cv::DMatch>> matches;
  matcher->knnMatch(descriptors, reference.descriptors, matches, 2);

  std::vector<cv::DMatch> filteredMatches;

  // Ratio test, identify strong key points that have a first match that is
  // closer than 75% of its second match. Collect the corresponding key points
  // that are a good match
  static constexpr double ratioTest = 0.75;
  for (const std::vector<cv::DMatch>& matchPair : matches) {
    if (matchPair[0].distance < ratioTest * matchPair[1].distance) {
      filteredMatches.emplace_back(matchPair[0]);
    }
  }

  // If match is better than existing, replace
  if (filteredMatches.size() > bestMatchesCount) {
    bestMatchesCount = filteredMatches.size();
    std::vector<cv::Point2f> pointsImage;
    std::vector<cv::Point2f> pointsReference;
    for (const cv::DMatch& match : filteredMatches) {
      pointsImage.emplace_back(keyPoints[match.queryIdx].pt);
      pointsReference.emplace_back(reference.keyPoints[match.trainIdx].pt);
    }
    referenceAffine     = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat affine2Rows = cv::estimateAffine2D(pointsImage, pointsReference);
    affine2Rows.row(0).copyTo(referenceAffine.row(0));
    affine2Rows.row(1).copyTo(referenceAffine.row(1));
    bestReferenceImage = &reference;
  }
}

/**
 * @brief Get the total compounded affine transformation by stepping down the
 * tree of reference images
 *
 * @return cv::Mat
 */
cv::Mat Image::getTotalAffine() const {
  if (referenceAffine.empty()) {
    return cv::Mat::eye(3, 3, CV_64F);
  }

  cv::Mat bestReferenceAffine = bestReferenceImage->getTotalAffine();
  return bestReferenceAffine * referenceAffine;
}

/**
 * @brief Apply alignment generated in generateAlignment
 *
 * @return std::vector<double> minimum rectangle for cropping purposes: top,
 * right, bottom, left
 */
std::vector<double> Image::applyAlignment() {
  // Read from file if available
  if (totalAffine.empty()) {
    boost::filesystem::path featuresPath(path.string() + ".yml");
    totalAffine = getTotalAffine().rowRange(0, 2);
    spdlog::debug("Saving affine and features to {}", featuresPath);
    cv::FileStorage fs(featuresPath.string(), cv::FileStorage::WRITE);
    cv::write(fs, "Affine", totalAffine);
    cv::write(fs, "Key Points", keyPoints);
    cv::write(fs, "Descriptors", descriptors);
    fs.release();
  }

  // Find affine transformation between the key points
  // Assumes the perspective does not change
  cv::warpAffine(image, image, totalAffine, image.size());

  // Get bounding box
  std::vector<cv::Point2d> corners(4);
  corners[0] = cv::Point2d(0, 0);
  corners[1] = cv::Point2d(static_cast<double>(image.cols), 0);
  corners[2] = cv::Point2d(static_cast<double>(image.cols),
                           static_cast<double>(image.rows));
  corners[3] = cv::Point2d(0, static_cast<double>(image.rows));
  for (cv::Point2d& point : corners) {
    cv::Point2d original = point;

    point.x = original.x * totalAffine.at<double>(0, 0) +
              original.y * totalAffine.at<double>(0, 1) +
              totalAffine.at<double>(0, 2);
    point.y = original.x * totalAffine.at<double>(1, 0) +
              original.y * totalAffine.at<double>(1, 1) +
              totalAffine.at<double>(1, 2);
  }

  // Find smallest rectangle that fits in the transformed image
  std::vector<double> rect(4);
  rect[0] = MAX(corners[0].y, corners[1].y);  // Top
  rect[1] = MIN(corners[1].x, corners[2].x);  // Right
  rect[2] = MIN(corners[2].y, corners[3].y);  // Bottom
  rect[3] = MAX(corners[0].x, corners[3].x);  // Left

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
const cv::Mat& Image::computeGradient() {
  static constexpr int laplacianSize = 5;
  cv::Mat grey;
  cv::GaussianBlur(image, grey, cv::Size(laplacianSize, laplacianSize), 0);
  cv::cvtColor(grey, grey, cv::COLOR_BGR2GRAY);
  cv::Laplacian(grey, gradient, CV_64F, laplacianSize);
  gradient = cv::abs(gradient);

  static constexpr double blurSize = 2.5;
  cv::GaussianBlur(gradient, gradient, cv::Size(0, 0), blurSize);
  // cv::normalize(gradient, gradient, 0.0, 1.0, cv::NORM_MINMAX);
  return gradient;
}

/**
 * @brief Normalize the gradient per pixel to the stack
 *
 * @param maxGradient per pixel maximum of gradients in the stack
 */
void Image::normalizeGradient(const cv::Mat& maxGradient) {
  cv::divide(gradient, maxGradient, gradient);
}

/**
 * @brief Replace the image with a solid color with specified hue
 *
 * @param hue
 */
void Image::colorize(const double hue) {
  image = cv::Scalar(hue, std::numeric_limits<uint8_t>::max(),
                     std::numeric_limits<uint8_t>::max());
  cv::cvtColor(image, image, cv::COLOR_HSV2BGR);
}

/**
 * @brief Save the image to a path
 *
 * @param dstPath to save image to
 * @param saveGradient will save the gradient image instead of the base if true
 */
void Image::save(const boost::filesystem::path& dstPath,
                 bool saveGradient) const {
  spdlog::info("Saving image to {}", dstPath);
  if (saveGradient) {
    cv::Mat tmp;
    tmp = gradient * std::numeric_limits<uint8_t>::max();
    if (!cv::imwrite(dstPath.string(), tmp)) {
      throw std::exception(
          ("Failed to save image to " + dstPath.string()).c_str());
    }
  } else {
    if (!cv::imwrite(dstPath.string(), image)) {
      throw std::exception(
          ("Failed to save image to " + dstPath.string()).c_str());
    }
  }
}

/**
 * @brief Combine images together using their gradient map
 *
 * @param dst destination output image
 * @param images list of images to combine
 */
void Image::combine(cv::Mat& dst, std::list<Image>& images) {
  // Gradient weighted
  for (int row = 0; row < images.front().image.rows; ++row) {
    for (int col = 0; col < images.front().image.cols; ++col) {
      double sumGradient = 0.0;
      cv::Vec3d pixel(0.0, 0.0, 0.0);
      for (const Image& image : images) {
        double val = pow(image.gradient.at<double>(row, col), 6.0);
        sumGradient += val;
        pixel += cv::Vec3d(image.image.at<cv::Vec3b>(row, col)) * val;
      }
      pixel                       = pixel / sumGradient;
      dst.at<cv::Vec3b>(row, col) = pixel;
    }
  }

  // Boolean maximum gradient combiner
  // for (int row = 0; row < images.front().image.rows; ++row) {
  //   for (int col = 0; col < images.front().image.cols; ++col) {
  //     double maxGradient = 0.0;
  //     cv::Vec3b pixel(0, 0, 0);
  //     for (const Image& image : images) {
  //       double val = image.gradient.at<double>(row, col);
  //       if (val > maxGradient) {
  //         pixel       = image.image.at<cv::Vec3b>(row, col);
  //         maxGradient = val;
  //       }
  //     }
  //     dst.at<cv::Vec3b>(row, col) = pixel;
  //   }
  // }
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