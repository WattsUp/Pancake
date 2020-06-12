#ifndef _PANCAKE_ARGUMENTS_H_
#define _PANCAKE_ARGUMENTS_H_

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <cxxopts.hpp>
#include <list>
#include <string>
#include <vector>

namespace pancake {

class Arguments {
 public:
  Arguments();

  void parse(int argc, char* argv[]);
  void helpAndExit();

  /**
   * @brief Get the files given from the command line
   *
   * @return const std::list<boost::filesystem::path>&
   */
  inline const std::list<boost::filesystem::path>& getFiles() const {
    return files;
  }

  /**
   * @brief Get the output file
   *
   * @return const boost::filesystem::path&
   */
  inline const boost::filesystem::path& getOutput() const { return output; }

  /**
   * @brief Output depth map or real image
   *
   * @return true if output should be a depth map
   * @return false otherwise
   */
  inline bool outputDepthMap() const { return depthMap; };

 private:
  void addPath(const boost::filesystem::path& path,
               const boost::regex& extFilter,
               bool recursive);

  cxxopts::Options options;

  std::list<boost::filesystem::path> files;
  boost::filesystem::path output;

  bool depthMap = false;
};

}  // namespace pancake

#endif /* _PANCAKE_ARGUMENTS_H_ */