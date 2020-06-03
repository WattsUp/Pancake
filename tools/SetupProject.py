#!/usr/bin/env python
## A script to check for all software dependencies (prompts for their
#  installation), modify top-level project name, modify targets, reset the git
#  repository to an initial commit, and tag the commit v0.0.0.

import Template

import argparse
import os
import pickle
import re
import shutil
import subprocess
import sys
import traceback

## Prompt the user to enter project name, and targets and their configuration
#  @return tuple (project name string, list of targets
#     (list: name string, windowsOnly boolean, winMain boolean))
def getConfig():
  projectName = input("Enter top level project name: ").lower().strip()
  projectName = re.sub(r" ", "-", projectName)

  targets = []
  more = True
  while more:
    name = input("----\nEnter target name: ").lower().strip().replace(" ", "-")
    windowsOnly = input("Will {} be Windows only? (Y/n): ".format(name)
                        ).lower().strip() == "y"
    if windowsOnly:
      name += "-win"
    winMain = input(
        "Will {} be use WinMain? (Y/n): ".format(name)).lower().strip() == "y"
    targets.append([name, windowsOnly, winMain])
    more = input(
      "Do you have more targets to add? (Y/n): ").lower().strip() == "y"
  return (projectName, targets)

## Write the top level CMakeList.txt with project name and targets
#  @param config tuple (project name string, list of targets
#     (list: name string, windowsOnly boolean, winMain boolean))
def writeTopCMakeList(config):
  with open("CMakeLists.txt", "r", newline="\n") as file:
    filedata = file.read()
    file.close()

  # Replace top level project name
  filedata = re.sub(r"project\(.*?\)",
                    "project(\"{}\")".format(config[0]), filedata)
  # Add targets to target list and subdirectory list
  targetList = "set (TARGETS_DIST\n"
  targetConfigList = "set (TARGETS_DISTCONFIG\n"
  subdirectories = "# Add each target subdirectory\n"
  for target in config[1]:
    # Name (-win for windows only)
    targetList += "  \"{}\"\n".format(target[0])
    if target[2]:  # Using WinMain for Windows configuration
      targetConfigList += "  \"WinMain\""
    else:
      targetConfigList += "  \"default\""
    targetConfigList += " # {}\n".format(target[0])
    subdirectories += "add_subdirectory(\"project-{}\")\n".format(target[0])
  targetList += ")"
  targetConfigList += ")"

  filedata = re.sub(r"set \(TARGETS_DIST\n(.*\n)*?\)", targetList, filedata)
  filedata = re.sub(r"set \(TARGETS_DISTCONFIG\n(.*\n)*?\)",
                    targetConfigList, filedata)
  filedata = re.sub(
      r"# Add each target subdirectory\n(add_subdirectory\(.*\"\)\n*)*",
      subdirectories + "\n",
      filedata)

  # Overwrite file
  with open("CMakeLists.txt", "w", newline="\n") as file:
    file.write(filedata)
    file.close()

  print("Modified ./CMakeLists.txt")


## Create a folder for each target
#  @param config tuple (project name string, list of targets
#     (list: name string, windowsOnly boolean, winMain boolean))
def createTargets(config):
  print("Removing existing project folders")
  for f in os.listdir("."):
    if re.match(r"project-.*", f):
      shutil.rmtree(f, ignore_errors=True)

  for target in config[1]:
    folder = "project-" + target[0]
    os.mkdir(folder)

    with open("tools/templates/CMakeLists.txt", "r", newline="\n") as file:
      data = file.read()
      file.close()
    data = re.sub(r"TARGET", target[0], data)
    if target[1]:  # windowsOnly
      data = re.sub(r"set \(SRCS_.*?\)\n\n", "", data, flags=re.S | re.M)
      data = re.sub(r"(target_sources\(.*?) \$<.*?.\)", r"\1)", data)
    else:
      if target[2]:  # winMain
        data = re.sub(r"  \"main\.cpp\"\n", "", data)
      else:
        data = re.sub(r"  \"main_.*\.cpp\"\n", "", data)

    path = os.path.join(folder, "CMakeLists.txt")
    with open(path, "w", newline="\n") as file:
      file.write(data)
      file.close()
    print("Created", path)

    if target[1]:  # windowsOnly
      path = os.path.join(folder, "main.cpp")
      if target[2]:  # winMain
        shutil.copy("tools/templates/main_win.cpp", path)
      else:
        shutil.copy("tools/templates/main_unix.cpp", path)
        print("Created", path)
    else:
      if (target[2]):  # winMain
        path = os.path.join(folder, "main_unix.cpp")
        shutil.copy("tools/templates/main_unix.cpp", path)
        print("Created", path)
        path = os.path.join(folder, "main_win.cpp")
        shutil.copy("tools/templates/main_win.cpp", path)
        print("Created", path)
      else:
        path = os.path.join(folder, "main.cpp")
        shutil.copy("tools/templates/main_unix.cpp", path)
        print("Created", path)


## Reset the git repository: remove current repo, initialize a new one, update
#  submodules, commit, tag
#  @param git executable
def resetGit(git):
  try:
    if os.path.exists(".git"):
      shutil.rmtree(".git", onerror=Template.chmodWrite)
    Template.call([git, "init"])
    print("Created new git repository")

    # Get list of previous submodules and add them to the fresh repository
    cmd = [git, "config", "-f", ".gitmodules",
           "--get-regexp", "^submodule\\..*\\.path$"]
    submodules = subprocess.check_output(cmd, universal_newlines=True).strip()

    for submodule in submodules.split("\n"):
      # Get the URL and local path of each submodule
      matches = re.search(r"^(submodule\..*\.)path (.*)$", submodule)
      path = matches[2]
      cmd = [git, "config", "-f", ".gitmodules", "--get", matches[1] + "url"]
      url = subprocess.check_output(cmd, universal_newlines=True).strip()

      shutil.rmtree(path, ignore_errors=True)
      Template.call([git, "submodule", "add", url, path])
      print("Added {} to {}".format(url, path))

    Template.call([git, "add", "."])

    Template.call([git, "commit", "-m", "Initial commit"])
    Template.call([git, "tag", "-a", "v0.0.0", "-m", "Initial release"])
    print("----\nCommitted master and tagged v0.0.0, need to push to remote")

  except Exception:
    print("Failed to reset git repository", file=sys.stderr)
    traceback.print_exc()
    sys.exit(1)

## Create documentation branch and commit template HTML website
#  @param git executable
#  @param documentation branch name
def createDocumentation(git, documentation):
  try:
    # Create orphaned documentation branch
    Template.call([git, "checkout", "--orphan", documentation, "--quiet"])
    Template.call([git, "reset", "."])

    # Move template files to top level
    for f in os.listdir("docs/templates/"):
      shutil.copy(
          Template.makeAbsolute(
              "docs/templates/" + f,
              os.getcwd()),
          os.path.join(os.getcwd(), f))

    # Add template files, commit
    Template.call([git, "add",
                   ".gitattributes", ".gitignore", ".nojekyll", "index.html"])
    Template.call([git, "commit", "-m", "Initial commit"])
    print("Committed documentation branch, need to push to remote")

    # Remove all other files
    Template.call([git, "clean", "-fd"])
    if os.path.exists("docs"):
      shutil.rmtree("docs", onerror=Template.chmodWrite)
    Template.call([git, "checkout", "master", "--quiet"])

  except Exception:
    print("Failed to reset git repository", file=sys.stderr)
    traceback.print_exc()
    sys.exit(1)


## Main function
def main():
  parser = argparse.ArgumentParser(description="Check for all software dependencies "
                                   "(prompts for their installation), modify top-level "
                                   "project name, modify targets, reset the git repository "
                                   "to an initial commit, and tag the commit v0.0.0.")
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
  parser.add_argument("--documentation-branch", default="gh-pages",
                      help="branch to create for documentation website")
  parser.add_argument("--discard-progress", action="store_true", default=False,
                      help="discard saved progress and start from step 1")
  parser.add_argument("--discard-config", action="store_true", default=False,
                      help="discard saved configuration and start from step 2")
  parser.add_argument("-s", default=None,
                      help="step to start with")

  argv = sys.argv[1:]
  args = parser.parse_args(argv)

  progress = Template.Progress()
  if not args.discard_progress:
    progress = progress.open()

  if args.s:
    progress.step = int(args.s) - 1

  print("----- Step 1: Software Check -----")
  if progress.step < 1:
    Template.checkInstallations(
        args.git,
        True,
        args.clang_format,
        args.clang_tidy,
        args.clang_apply_replacements,
        args.doxygen,
        args.cmake,
        True,
        False)
    progress.increment()
    print("All software dependencies have been installed")
  else:
    print("Previously completed")

  print("----- Step 2: Get Configuration -----")
  if progress.step < 2 or args.discard_config:
    progress.step = 1
    config = getConfig()
    progress.attachment = config
    progress.increment()
  else:
    print("Previously completed")
    config = progress.attachment

  print("----- Configuration -----")
  print("Project Name: \"{}\"".format(config[0]))
  for target in config[1]:
    message = "Target: \"{}\"".format(target[0])
    if target[1]:
      message += ", Windows only"
    else:
      message += ", Windows & UNIX"
    if target[2]:
      message += ", using WinMain() for Windows"
    else:
      message += ", using int main()"
    print(message)

  print("----- Step 3: Edit Top CMakeList -----")
  if progress.step < 3:
    writeTopCMakeList(config)
    progress.increment()
  else:
    print("Previously completed")

  print("----- Step 4: Create Targets -----")
  if progress.step < 4:
    createTargets(config)
    progress.increment()
  else:
    print("Previously completed")

  print("----- Step 5: Initialize git Repository -----")
  if progress.step < 5:
    resetGit(args.git)
    progress.increment()
  else:
    print("Previously completed")

  print("----- Step 6: Initialize Documentation Branch -----")
  if progress.step < 6:
    createDocumentation(args.git, args.documentation_branch)
    progress.increment()
  else:
    print("Previously completed")

  print("----- Step 7: Clean -----")
  if progress.step < 7:
    Template.call([args.git, 'clean', '-xfd'])
    progress.increment()
  else:
    print("Previously completed")

  progress.complete()
  print("Make sure to push with 'git   push --tags'")


if __name__ == "__main__":
  main()
