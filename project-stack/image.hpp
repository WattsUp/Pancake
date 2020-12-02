#ifndef _PANCAKE_IMAGE_H_
#define _PANCAKE_IMAGE_H_

#include <boost/filesystem.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>

namespace pancake {

class Image {
 public:
  Image(const boost::filesystem::path& path);

  bool detectFeatures(const cv::Ptr<cv::Feature2D>& feature2D);
  void generateBestAlignment(const cv::Ptr<cv::DescriptorMatcher>& matcher,
                             const Image& reference);
  std::vector<double> applyAlignment();

  void crop(const std::vector<double>& rect);

  const cv::Mat& computeGradient();
  void normalizeGradient(const cv::Mat& maxGradient);
  void colorize(const double hue);

  void save(const boost::filesystem::path& path,
            bool saveGradient = false) const;

  /**
   * @brief Get the type of image stored
   *
   * @return int OpenCV image types: CV_8U...
   */
  inline int type() const { return image.type(); }

  /**
   * @brief Get the size of image stored
   *
   * @return cv::Size
   */
  inline cv::Size size() const { return image.size(); }

  /**
   * @brief Get the name of the image stored
   *
   * @return const std::string&
   */
  inline std::string getName() const { return path.filename().string(); };

  static void combine(cv::Mat& dst, std::list<Image>& images);

 private:
  cv::Mat getTotalAffine() const;

  const boost::filesystem::path path;

  cv::Mat image;
  cv::Mat gradient;

  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;

  const Image* bestReferenceImage = nullptr;
  size_t bestMatchesCount         = 0;
  cv::Mat referenceAffine;
  cv::Mat totalAffine;
};

}  // namespace pancake

#endif /* _PANCAKE_IMAGE_H_ */