# Pancake #
Photography image stacker for use in extended depth of field of macro images

## Building ##
### Dependencies ###
On Windows I recommend using [vcpkg](https://github.com/Microsoft/vcpkg)
* [spdlog](https://github.com/gabime/spdlog) logging library
* [Google Test](https://github.com/google/googletest) framework
* [OpenImageIO](https://github.com/OpenImageIO/oiio)
* [cxxopts](https://github.com/jarro2783/cxxopts) command line parser

### Git Clone ###
Clone the repository
```bash
> mkdir workspace
> cd workspace
> git clone https://github.com/WattsUp/Pancake
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
* `project-stack` Command line application to normalize, register, align, and stack images together
