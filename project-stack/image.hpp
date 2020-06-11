#ifndef _PANCAKE_IMAGE_H_
#define _PANCAKE_IMAGE_H_

#include <boost/filesystem.hpp>
#include <opencv2/core/mat.hpp>

namespace pancake {

class Image {
 public:
  Image(const boost::filesystem::path& path);

  void save(const boost::filesystem::path& path) const;

 private:
  cv::Mat image;
};

}  // namespace pancake

#endif /* _PANCAKE_IMAGE_H_ */