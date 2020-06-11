#ifndef _PANCAKE_ARGUMENTS_H_
#define _PANCAKE_ARGUMENTS_H_

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <list>
#include <string>
#include <vector>

namespace pancake {

class Arguments {
 public:
  Arguments() = default;

  void parse(int argc, char* argv[]);

  /**
   * @brief Get the files given from the command line
   *
   * @return const std::list<boost::filesystem::path>&
   */
  inline const std::list<boost::filesystem::path>& getFiles() const {
    return files;
  }

 private:
  void addPath(const boost::filesystem::path& path,
               const boost::regex& extFilter,
               bool recursive);

  std::list<boost::filesystem::path> files;
};

}  // namespace pancake

#endif /* _PANCAKE_ARGUMENTS_H_ */