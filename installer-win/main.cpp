#include "common/Version.h"
#include "common/logging.hpp"

#include "Resources.h"

#include <Windows.h>
#include <miniz.h>
#include <cstdio>
#include <exception>
#include <fstream>

/**
 * @brief Extract the archive to the install folder
 *
 * @param path to save the extracted contents to
 * @return bool true when extraction was successful
 */
bool extractArchive(char* path) {
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
  try {
#if DEBUG
    spdlog::info(VERSION_STRING_FULL);
    spdlog::set_level(spdlog::level::debug);
#else  /* DEBUG */
    spdlog::info(VERSION_STRING);
#endif /* DEBUG */

    for (int i = 0; i < argc; ++i) {
      // NOLINTNEXTLINE (cppcoreguidelines-pro-bounds-pointer-arithmetic)
      spdlog::info("Argument: {}", argv[i]);
    }

    if (!extractArchive("temp")) {
      spdlog::error("Error occurred whilst extracting archive");
    }
  } catch (std::exception& e) {
    // Catch exceptions from spdlog
    puts(e.what());
  }

  return 0;
}
