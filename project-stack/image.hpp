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
  void applyAlignment();

  void save(const boost::filesystem::path& path) const;

 private:
  static bool compareMatches(const cv::DMatch& i, const cv::DMatch& j);

  cv::Mat image;

  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
  std::vector<cv::Point2f> pointsImage;
  std::vector<cv::Point2f> pointsReference;
};

}  // namespace pancake

#endif /* _PANCAKE_IMAGE_H_ */