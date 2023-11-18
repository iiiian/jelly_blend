# Jelly Blend - A soft Body simulation addon for Blender

## Install

## Uninstall

## Build from source

### Linux

1. install build tools and dependencies

   - a complier (ex. `gcc`/`clang`)
   - a cmake generator (ex. `make`/`ninja`)
   - python 3.10
   - `cmake`
   - `git`

   you can install these build tools on Ubuntu by 

   ```
   sudo apt install build-essential cmake git
   ```

   check your system python version

   ```
   python3 --version
   ```

   if the version is not 3.10.x, you have to setup the correct version of python. I recommend python PPA or `miniconda`.

2. clone the repo and configure
   ```
   git clone https://github.com/iiiian/jelly_blend.git
   ```
	create build directory and configure
   ```
   cd jelly_blend
   mkdir build
   cd build
   cmake ..
   ```
	you might need to give `cmake` a hint to the correct python installation
	```
	cmake -DPython_ROOT_DIR="path to your python binary" ..
   ```


3. build and install
   ```
   cmake --build .
   ```

   after the build, there will be a folder called `jelly_blend_addon`. To install the addon, you can move the folder to the blender addon directory. Which is typically located at `~/.config/blender/3.6/scripts/addons`. Or you can compress the folder to `.zip` format and install the addon from blender gui.

### Windows

1. install build tools and dependencies

   - a complier (MSVC)
     I do not suggest using `Mingw`, since you will need to link standard libraries statically.
   - a cmake generator (ex. `make`/`ninja`)
   - python 3.10
   - `cmake`
   - `git`

   The most convenient way to get these build tools is to install the `Desktop development with C++` kit in Visual Studio. As for python, `miniconda/anaconda` is strongly recommended.

2. clone the repo and configure

   ```
   git clone https://github.com/iiiian/jelly_blend.git
   ```

   create build directory and configure

   ```
   cd jelly_blend
   mkdir build
   cd build
   cmake ..
   ```

   you might need to give `cmake` a hint to the correct python installation

   ```
   cmake -DPython_ROOT_DIR="path to your python binary" ..
   ```

3. build and install

   ```
   cmake --build .
   ```

   after the build, there will be a folder called `jelly_blend_addon`. To install the addon, you can move the folder to the blender addon directory. Or you can compress the folder to `.zip` format and install the addon from blender gui.

