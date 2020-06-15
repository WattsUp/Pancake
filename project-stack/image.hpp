#ifndef _PANCAKE_IMAGE_H_
#define _PANCAKE_IMAGE_H_

#include <boost/filesystem.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>

namespace pancake {

class Image {
 public:
  Image(const boost::filesystem::path& path);

  void detectFeatures(const cv::Ptr<cv::Feature2D>& feature2D);
  size_t generateAlignment(const cv::Ptr<cv::DescriptorMatcher>& matcher,
                           const Image& reference);
  std::vector<double> applyAlignment();

  void crop(const std::vector<double>& rect);

  double computeGradient();
  const cv::Mat& quantizeGradient(const double maxGradientVal);
  void combineByMaskGradient(cv::Mat& dst, const cv::Mat& maxGradient);
  void colorize(const double hue);

  void save(const boost::filesystem::path& path,
            bool saveGradient = false) const;

  /**
   * @brief Get the type of image stored
   *
   * @return int OpenCV image types: CV_8U...
   */
  inline int type() const { return image.type(); }

 private:
  static bool compareMatches(const cv::DMatch& i, const cv::DMatch& j);

  const boost::filesystem::path path;

  cv::Mat image;
  cv::Mat gradient;

  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
  std::vector<cv::Point2f> pointsImage;
  std::vector<cv::Point2f> pointsReference;
};

}  // namespace pancake

#endif /* _PANCAKE_IMAGE_H_ */