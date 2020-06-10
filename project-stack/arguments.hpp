#ifndef _PANCAKE_ARGUMENTS_H_
#define _PANCAKE_ARGUMENTS_H_

#include <list>
#include <string>

namespace pancake {

class Arguments {
 public:
  Arguments() = default;

  void parse(int argc, char* argv[]);

 private:
  std::list<std::string> files;
};

}  // namespace pancake

#endif /* _PANCAKE_ARGUMENTS_H_ */