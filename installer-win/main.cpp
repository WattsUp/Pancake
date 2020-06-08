#include "common/Version.h"
#include "common/logging.hpp"

#include "Resources.h"

#include <Windows.h>
#include <miniz.h>
#include <cstdio>
#include <cxxopts.hpp>
#include <exception>
#include <fstream>

/**
 * @brief Extract the archive to the install folder
 *
 * @param path to save the extracted contents to
 * @return bool true when extraction was successful
 */
bool extractArchive(const char* path) {
  // NOLINTNEXTLINE (cppcoreguidelines-pro-type-cstyle-cast)
  HRSRC res = ::FindResource(nullptr, MAKEINTRESOURCE(RES_ARCHIVE), RT_RCDATA);
  HGLOBAL data = ::LoadResource(nullptr, res);
  void* pData  = ::LockResource(data);
  size_t size  = ::SizeofResource(nullptr, res);

  spdlog::info("Extracting archive to {}", path);
  if ((CreateDirectoryA(path, nullptr) == 0) &&
      GetLastError() != ERROR_ALREADY_EXISTS) {
    spdlog::error("Failed to make directory {}: {}", path, errno);
    return false;
  }

  mz_zip_archive zip;
  memset(&zip, 0, sizeof(zip));
  if (mz_zip_reader_init_mem(&zip, pData, size, 0) == 0) {
    spdlog::error("Failed opening archive");
    return false;
  }

  spdlog::debug("Archive contains {} files", mz_zip_reader_get_num_files(&zip));

  for (mz_uint i = 0; i < mz_zip_reader_get_num_files(&zip); i++) {
    mz_zip_archive_file_stat stat;
    if (mz_zip_reader_file_stat(&zip, i, &stat) == 0) {
      spdlog::error("Failed getting file {}'s statistics", i);
      mz_zip_reader_end(&zip);
      return false;
    }
    spdlog::debug("\"{}\" {}B => {}B", stat.m_filename, stat.m_comp_size,
                  stat.m_uncomp_size);

    std::string filePath =
        std::string{path} + "/" + static_cast<char*>(stat.m_filename);
    if (mz_zip_reader_is_file_a_directory(&zip, i) != 0) {
      if ((CreateDirectoryA(filePath.c_str(), nullptr) == 0) &&
          GetLastError() != ERROR_ALREADY_EXISTS) {
        spdlog::error("Failed to make directory {}: {}", path, errno);
        return false;
      }
    } else {
      if (mz_zip_reader_extract_to_file(&zip, i, filePath.c_str(), 0) == 0) {
        spdlog::error("Failed writing file {} to {}", i, filePath.c_str());
        mz_zip_reader_end(&zip);
        return false;
      }
    }
  }

  mz_zip_reader_end(&zip);

  return true;
}

/**
 * @brief Main entry point for program
 *
 * @param argc count of arguments
 * @param argv array of arguments
 * @return int zero on success, non-zero on failure
 */
int main(int argc, char* argv[]) {
  std::string installDirectory;

  try {
    cxxopts::Options options("Pancake Installer",
                             "Intall Pancake - a photography stacking tool");
    options.positional_help("<Extra args>");

    cxxopts::OptionAdder optionsAdder = options.add_options();
    optionsAdder("v,verbose", "Verbose output",
                 cxxopts::value<bool>()->default_value("false"));
    optionsAdder("d,dir", "Install directory",
                 cxxopts::value<std::string>()->default_value("temp"));
    optionsAdder("version", "Software version",
                 cxxopts::value<bool>()->default_value("false"));
    optionsAdder("h,help", "Print usage");

    cxxopts::ParseResult result = options.parse(argc, argv);

    if (result.count("help") != 0) {
      std::cout << options.help() << std::endl;
      return 0;
    }

    if (result.count("version") != 0) {
#if DEBUG
      std::cout << VERSION_STRING_FULL << std::endl;
#else  /* DEBUG */
      std::cout << VERSION_STRING << std::endl;
#endif /* DEBUG */
      return 0;
    }

#if DEBUG
    spdlog::set_level(spdlog::level::info);
#else  /* DEBUG */
    spdlog::set_level(spdlog::level::warn);
#endif /* DEBUG */
    if (result.count("verbose") != 0 && result["verbose"].as<bool>()) {
#if DEBUG
      spdlog::set_level(spdlog::level::debug);
#else  /* DEBUG */
      spdlog::set_level(spdlog::level::info);
#endif /* DEBUG */
    }

    installDirectory = result["dir"].as<std::string>();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  try {
    spdlog::info("Installing Pancake to {}", installDirectory);
    if (!extractArchive(installDirectory.c_str())) {
      spdlog::error("Error occurred whilst extracting archive");
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  return 0;
}
