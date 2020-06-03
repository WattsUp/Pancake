#!/usr/bin/env python
## Shared functions the template tools use
import sys
if sys.version_info[0] != 3 or sys.version_info[1] < 6:
  print("This script requires Python version >=3.6")
  sys.exit(1)

import argparse
import os
import pickle
import re
import subprocess
import stat
import traceback

## Semantic versioning object with string parsing
class Version:
  string = ""
  major = 0
  minor = 0
  patch = 0
  tweak = ""
  ahead = 0
  modified = False
  gitSHA = ""

  ## Initialization of Version object
  #  @param self object pointer
  #  @param string of version: "[major].[minor].[patch](-tweak)"
  #  @param ahead number of commits ahead of version string
  #  @param modified state of the repository
  #  @param gitSHA of current repository state
  def __init__(self, string, ahead=0, modified=False, gitSHA=""):
    self.string = string
    self.ahead = ahead
    self.modified = modified
    self.gitSHA = gitSHA

    versionList = re.search(
        r"(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(-[a-zA-Z\d][-a-zA-Z.\d]*)?", string)
    self.major = int(versionList[1])
    self.minor = int(versionList[2])
    self.patch = int(versionList[3])
    if versionList[4]:
      self.tweak = versionList[4]
    else:
      self.tweak = ""

  ## String representation of Version
  #  @param self object pointer
  #  @return version string "[major].[minor].[patch](-tweak)"
  def __str__(self):
    return self.string

  ## Full string representation of Version
  #  @param self object pointer
  #  @return full version string "[major].[minor].[patch](-tweak)+[modified][ahead].[gitSHA]"
  def fullStr(self):
    if self.modified:
      return "{}+~{}.{}".format(self.string, self.ahead, self.gitSHA)
    return "{}+{}.{}".format(self.string, self.ahead, self.gitSHA)

  ## Greater than or equal to comparison
  #  @param self object pointer
  #  @param other object pointer
  #  @return true if self's version is higher than other's, false otherwise
  def __ge__(self, other):
    if self.major < other.major:
      return False
    if self.minor < other.minor:
      return False
    if self.patch < other.patch:
      return False
    return True

## Run a command, read its output for semantic version, compare to a minimum
#  @param cmd command to run, i.e. ["git", "--version"]
#  @param minimum semantic version string to compare to
#  @return true if outputted version is greater or equal to the minimum,
#    false otherwise (including exception occurred whilst executing command)
def checkSemver(cmd, minimum):
  try:
    output = subprocess.check_output(
        cmd, universal_newlines=True)
  except BaseException:
    sys.stderr.write(
        "Unable to run {:}. Is command correctly specified?\n".format(cmd[0]))
    return False
  matches = re.search(
      r"(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)",
      output,
      flags=re.S)
  version = Version(matches.expand(r"\1.\2.\3"))

  return version >= Version(minimum)

## Run a command in a directory (optional), redirect stdout to DEVNULL
#  @param cmd command to run, i.e. ["git", "--version"]
#  @param cwd directory to run command in
#  @return return code of the command
def call(cmd, cwd="."):
  return subprocess.check_call(cmd, cwd=cwd, stdout=subprocess.DEVNULL)

## Modify a path for write access, usually called on error of func
#  @param func object pointer
#  @param path to update permissions
#  @param excinfo
def chmodWrite(func, path, excinfo):
  os.chmod(path, stat.S_IWRITE)
  func(path)

## Make a path absolute
#  @param f path to check
#  @param directory path is relative to
#  @return absolute path of f
def makeAbsolute(f, directory):
  if os.path.isabs(f):
    return os.path.abspath(f)
  return os.path.normpath(os.path.join(directory, f))

## Find the file, checks parent folders until root
#  @param file to find
#  @return absolute path of file
def findInParent(file, directory):
  while not os.path.isfile(os.path.join(directory, file)):
    if os.path.normpath(directory) == "/":
      print("Error: could not file:", file)
      sys.exit(1)
    directory += "../"
  return makeAbsolute(file, directory)

## Write data to file if file does not exist or existing content is different
#  @param path to write to
#  @param data to write
#  @param quiet will only print errors
def overwriteIfChanged(path, data, quiet):
  write = True
  if os.path.isfile(path):
    with open(path, "r", newline="\n") as file:
      write = file.read() != data
      if not write and not quiet:
        print("File unchanged:", path)
  if write:
    with open(path, "w", newline="\n") as file:
      file.write(data)
      if not quiet:
        print("Wrote to:", path)

## Class to track the progress of a procedure between runs of the script
class Progress:
  ## Initialize a progress object
  #  @param self object pointer
  #  @param step of current progress
  #  @param path to save progress object to
  def __init__(self, step=0, path='progress.pkl'):
    self.step = step
    self.path = makeAbsolute(path, os.getcwd())
    self.attachment = None

  ## Open progress from save file
  #  @param self object pointer
  #  @return Progress object loaded from save if path exists
  def open(self):
    if os.path.exists(self.path):
      with open(self.path, 'rb') as file:
        return pickle.load(file)
    return self

  ## Increment progress and save progress to file
  #  @param self object pointer
  def increment(self):
    self.step += 1
    with open(self.path, 'wb') as file:
      pickle.dump(self, file, pickle.HIGHEST_PROTOCOL)

  ## Complete the procedure by deleting the progress save file
  #  @param self object pointer
  def complete(self):
    if os.path.exists(self.path):
      os.remove(self.path)

## Check the required installations
#  If an installation does not pass check, terminate program
#  @param git executable
#  @param gitConfig will check for user.name and user.email when true
#  @param clangFormat executable
#  @param clangTidy executable
#  @param clangApplyReplacements executable
#  @param doxygen executable
#  @param cmake executable
#  @param testCompiler will test for a compiler when true
#  @param quiet will only print errors
def checkInstallations(git=None, gitConfig=False, clangFormat=None, clangTidy=None, clangApplyReplacements=None,
                       doxygen=None, cmake=None, testCompiler=False, quiet=False):
  if git:
    if not quiet:
      print("Checking git version")
    if not checkSemver([git, "--version"], "2.17.0"):
      print("Install git version 2.17+", file=sys.stderr)
      sys.exit(1)

    if gitConfig:
      if not quiet:
        print("Checking git config")
      try:
        call([git, "config", "--global", "user.name"])
        call([git, "config", "--global", "user.email"])
      except Exception:
        print("No identity for git", file=sys.stderr)
        print("git config --global user.name \"Your name\"", file=sys.stderr)
        print(
            "git config --global user.email \"you@example.com\"",
            file=sys.stderr)
        sys.exit(1)

  if clangFormat:
    if not quiet:
      print("Checking clang-format version")
    if not checkSemver([clangFormat, "--version"], "7.0.0"):
      print("Install clang-format version 7.0+", file=sys.stderr)
      sys.exit(1)

  if clangTidy:
    if not quiet:
      print("Checking clang-tidy version")
    if not checkSemver([clangTidy, "--version"], "7.0.0"):
      print("Install clang-tidy version 7.0+", file=sys.stderr)
      sys.exit(1)

  if clangApplyReplacements:
    if not quiet:
      print("Checking clang-apply-replacements version")
    if not checkSemver([clangApplyReplacements, "--version"], "7.0.0"):
      print("Install clang-apply-replacements version 7.0+", file=sys.stderr)
      sys.exit(1)

  if doxygen:
    if not quiet:
      print("Checking doxygen version")
    if not checkSemver([doxygen, "--version"], "1.8.17"):
      print("Install doxygen version 1.8.17+", file=sys.stderr)
      sys.exit(1)

  if cmake:
    if not quiet:
      print("Checking cmake version")
    if not checkSemver([cmake, "--version"], "3.13.0"):
      print("Install cmake version 3.13+", file=sys.stderr)
      sys.exit(1)

    if testCompiler:
      if not quiet:
        print("Checking working compiler exists")
      try:
        call([cmake, "-E", "make_directory", "__temp__"])
        call([cmake, "-E", "touch", "CMakeLists.txt"], "__temp__")
        call([cmake, "--check-system-vars", "-Wno-dev", "."], "__temp__")
        call([cmake, "-E", "remove_directory", "__temp__"])
      except Exception:
        print("Failed to check for a compiler", file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      description="Check for all software dependencies")
  parser.add_argument("--cmake", metavar="PATH", default="cmake",
                      help="path to cmake binary")
  parser.add_argument("--clang-format", metavar="PATH", default="clang-format",
                      help="path to clang-format binary")
  parser.add_argument("--clang-tidy", metavar="PATH", default="clang-tidy",
                      help="path to clang-tidy binary")
  parser.add_argument("--clang-apply-replacements", metavar="PATH", default="clang-apply-replacements",
                      help="path to clang-apply-replacements binary")
  parser.add_argument("--git", metavar="PATH", default="git",
                      help="path to git binary")
  parser.add_argument("--doxygen", metavar="PATH", default="doxygen",
                      help="path to doxygen binary")

  argv = sys.argv[1:]
  args = parser.parse_args(argv)

  checkInstallations(
      args.git,
      True,
      args.clang_format,
      args.clang_tidy,
      args.clang_apply_replacements,
      args.doxygen,
      args.cmake,
      True,
      False)
  print("All software dependencies have been installed")
