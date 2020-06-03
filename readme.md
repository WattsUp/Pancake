# C++ Project Template #
A template repository designed for medium to large C++ projects with multiple subprojects/executables.
Includes
* CMake build system
* Automatic semantic versioning
* [spdlog](https://github.com/gabime/spdlog) logging library
* [Google Test](https://github.com/google/googletest) framework
* Clang-format and clang-tidy
* Multiple target executables for main program, installer, auto-updater, tester, etc.
* Doxygen documentation

## Prerequisites ##
* Modern C/C++ compiler
* [git 2.17+](https://git-scm.com/downloads) installed
* [CMake 3.13+](https://cmake.org/download/) installed
* [Clang tools 7.0+](http://releases.llvm.org/download.html) namely clang-format and clang-tidy
* [Python 3.6+](https://www.python.org/downloads/) for tools
* [Doxygen 1.8.17+](http://www.doxygen.nl/download.html) for documentation generation

**Recommended IDE** is [VSCode](https://code.visualstudio.com/) with the following extensions
* [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) language support
* [Clang-Format](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format) integration
* [CMake](https://marketplace.visualstudio.com/items?itemName=twxs.cmake) language support
* [CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)
* [Code Spell Checker](https://marketplace.visualstudio.com/items?itemName=streetsidesoftware.code-spell-checker)
* [Doxygen Documentation Generator](https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen)

## Building ##
### Git Clone ###
Clone the repository and its submodules
```bash
> mkdir workspace
> cd workspace
> git clone --recurse-submodules https://github.com/WattsUp/CppProjectTemplate myProject
```

### Manually Building ###
Configure the project with default compiler and compile
```bash
> cmake . -B build
> cmake --build build
```

### Building in VSCode ###
1. Open the project folder in VSCode with the recommended extensions
2. Open the CMake Tools tab
3. Select `Configure All Projects`, select appropriate compiler
4. Click `Build All Projects`

## Folder Structure ##
* `bin`       Binary folder, output directory for executables, add runtime resources here (icons, etc.)
* `common`    Common code shared amongst projects: logging, utilities, etc.
* `docs`      Documentation folder
* `docs\www`  Documentation webpage root folder, ignored, clone of repository's gh-pages branch
* `include`   Public include folder for libraries
* `libraries` Third party libraries usually included as a `git submodule`
* `tools`     Helper code such as check coding conventions script

### Projects ###
* `project-installer` Installer executable which sets up directories, variables, etc.
* `project-fractal`   Application to draw a fractal to the console

## Adapting ##
1. Clone the repository, see [Git Clone](#git-clone). (Or download manually)
2. Run the python script `tools/SetupProject.py` from the top level folder. It will guide you through the process. The script checks for all software dependencies (prompts for their installation), modifies top-level project name, modifies targets, resets the git repository to an initial commit, and tags the commit v0.0.0. The only dependency is the ability to run python scripts.

### Modify Project Name ###
Change `project("project-template")` to `project("[top-level-project-name]")` in `./CMakeLists.txt`

### Modify Sub Projects and Targets ###
1. Modify/create folders for each sub project, see [Folder Structure](#folder-structure)
2. Modify/create target names in all `CMakeLists.txt`:
```CMake
set (TARGETS
  "[target-name-1]" # [Optional comment]
  "[target-name-2]" # [Optional comment]
)
```
and
```CMake
target_sources("[Target name 1]" PRIVATE ${SRCS})
```
**Note:** Target names cannot have spaces in CMake. To have spaces in the output files uncomment the following in `./CMakeLists.txt` and duplicate as needed:
```CMake
set_target_properties([TargetNoSpaces] PROPERTIES OUTPUT_NAME "Target With Spaces")
```
3. Add the appropriate subdirectories to `./CMakeLists.txt`
```CMake
# Add each subdirectory
add_subdirectory("common")
add_subdirectory("project-installer")
add_subdirectory("project-fractal")
```

### Documentation branch ###
The `docs/www` folder contains the documentation branch of this repo (`gh-pages` for GitHub). Once the repository has a remote, clone that branch into that folder and use it to build a documentation website.
```bash
git clone [URL] -b gh-pages docs/www
```