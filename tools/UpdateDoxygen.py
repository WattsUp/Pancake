#!/usr/bin/env python
## A script to update the webpage folder with doxygen generated documentation

import Template
import CreateVersionFromGitTag as VersionTag

import argparse
import os
import re
import shutil
import subprocess
import sys
import traceback

## Checks current branch is the default branch, up to date with remote,
#  unmodified, and tagged
#  @param git executable
#  @param quiet true will only print errors
#  @param version Template.Version object of the repository
#  @return true when current branch meets criteria, false otherwise
def checkCurrentBranchStatus(git, quiet, version):
  status = True

  # Fetch remote
  Template.call([git, "fetch"])

  # Get name of default branch
  cmd = [git, "symbolic-ref", "refs/remotes/origin/HEAD"]
  default = subprocess.check_output(cmd, universal_newlines=True).strip()
  default = re.match(r"refs/remotes/origin/(.*)", default).group(1)

  # Get branch status compared to remote
  cmd = [git, "branch", "-vv"]
  string = subprocess.check_output(cmd, universal_newlines=True).strip()
  match = re.search(
      r"^\* ([^\s]*) *[^\s]* *\[origin/([^\s:]*):? ?((ahead|behind) .*)?\]",
      string,
      flags=re.M)
  if not match:
    print("Current branch does not have a remote", file=sys.stderr)
    status = False
  else:
    groups = match.groups()
    if groups[1] != default:
      print("Current branch is not default (origin/{})".format(default),
            file=sys.stderr)
      status = False
    if groups[2]:
      print("Current branch is {} of its remote".format(groups[2]),
            file=sys.stderr)
      status = False

  if version.modified:
    print("Repository is modified", file=sys.stderr)
    status = False

  if version.ahead:
    print("Repository is {} commits ahead of a tag".format(version.ahead),
          file=sys.stderr)
    status = False

  return status

## Main function
def main():
  # Create an arg parser menu and grab the values from the command arguments
  parser = argparse.ArgumentParser(description="Update the webpage folder with"
                                   " doxygen generated documentation")
  parser.add_argument("--git", metavar="PATH", default="git",
                      help="path to git binary")
  parser.add_argument("--doxygen", metavar="PATH", default="doxygen",
                      help="path to doxygen binary")
  parser.add_argument("--quiet", action="store_true", default=False,
                      help="only output return codes and errors")
  parser.add_argument("--doxygen-output", metavar="PATH", required=True,
                      help="output file to write project info for doxygen")
  parser.add_argument("-f", action="store_true", default=False,
                      help="ignore current branch status check: default "
                      "branch, up to date with remote, unmodified, and tagged")
  parser.add_argument("--project-name", required=True,
                      help="name of project to add to generated documentation")
  parser.add_argument("--project-brief", required=True,
                      help="brief of project to add to generated documentation")

  argv = sys.argv[1:]
  args = parser.parse_args(argv)

  Template.checkInstallations(
      git=args.git,
      doxygen=args.doxygen,
      quiet=args.quiet)

  try:
    version = VersionTag.getVersion(args.git)
  except Exception:
    print("Exception getting version from git tags", file=sys.stderr)
    traceback.print_exc()
    sys.exit(1)

  try:
    if not checkCurrentBranchStatus(args.git, args.quiet, version):
      if not args.f:
        print("Current repository is not updated default branch", file=sys.stderr)
        sys.exit(1)
  except Exception:
    print("Exception checking branch status", file=sys.stderr)
    traceback.print_exc()

  data = f"""PROJECT_NAME           = "{args.project_name}"
PROJECT_NUMBER         = "{version.string}"
PROJECT_BRIEF          = "{args.project_brief}"
"""
  Template.overwriteIfChanged(args.doxygen_output, data, args.quiet)

  if os.path.exists("docs/www/doxygen"):
    shutil.rmtree("docs/www/doxygen", onerror=Template.chmodWrite)
  try:
    cmd = [args.doxygen, "docs/doxyfile"]
    Template.call(cmd)
  except Exception:
    print("Exception running doxygen", file=sys.stderr)
    traceback.print_exc()
    sys.exit(1)


if __name__ == "__main__":
  main()
