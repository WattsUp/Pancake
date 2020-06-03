#include <gtest/gtest.h>

#include "version.h"

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  std::cerr << "[          ] "
            << "Project Version: " << VERSION_STRING_FULL << std::endl;
  return RUN_ALL_TESTS();
}