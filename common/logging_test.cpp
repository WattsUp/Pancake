#include "logging.hpp"

#include <gtest/gtest.h>

TEST(Logging, NoSinks) {
  common::logging::configure(nullptr, false);
  EXPECT_EQ(size_t{0}, spdlog::default_logger()->sinks().size());
}

TEST(Logging, FileSink) {
  common::logging::configure("log.log", false);
  EXPECT_EQ(size_t{1}, spdlog::default_logger()->sinks().size());
}

TEST(Logging, ConsoleSink) {
  common::logging::configure(nullptr, true);
  EXPECT_EQ(size_t{1}, spdlog::default_logger()->sinks().size());
}

TEST(Logging, FileAndConsoleSink) {
  common::logging::configure("log.log", true);
  spdlog::debug("debug Test");
  spdlog::info("info Test");
  spdlog::warn("warn Test");
  spdlog::error("error Test");
  spdlog::critical("critical Test");
  EXPECT_EQ(size_t{2}, spdlog::default_logger()->sinks().size());
}