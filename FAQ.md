# FAQ

## Q: After upgrading PCL from 1.X to 1.Y, my code is crashing inexplicably!*

A: It is possible that you are using one version of the headers and a different version of the libraries. First, check your CMakeLists.txt to ensure  it is searching for the right version:
```
FIND_PACKAGE(PCL 1.Y REQUIRED)
```
This line tells CMake which headers to use. If you specify the wrong version, it will use old headers with new libraries, which is very bad.

To make things easier, if possible, ensure the old version has been uninstalled (eg: by using `make uninstall` if you installed from source). If not, manually delete the *include/pcl-1.X folder*, and all .so files ending in 1.X.


## Q: My Qt Application exits if I use the PCLVisualizer class and close it's view window, what shall I do?*

A: This problem seems to be related to the spin()/spinOnce() function of PCLVisualizer as it registers some callbacks that might exit the whole application.
As a work around in a Qt based application you can replace the
```
while (!viewer->wasStopped())
        viewer->spinOnce(100);
```
with
```
while (!viewer->wasStopped())
        qApp->processEvents();
```
to make use of the Qt main event loop for processing the viewer events.

## Q: I want to use `pcl::NormalEstimationOMP` but setting the number of threads/cores via `setNumberOfThreads` doesn't seem to work.*

A: Try adding the `openmp` flag to your compiler, e.g.:

```
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
```

## Q: How do I use a different version of PCL with ROS?

A: If you intend to use PCL with some custom flag or version that is not shipped by your package manager, you need to ensure that:
1. All packages which rely on PCL are built from source
2. ROS toolchain can access your custom build of PCL

For point 1, we recommend using the overlay feature of both [catkin](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying) and [colcon](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#source-an-underlay). This ensures that even packages that might have been installed by the package manager are not used in favour of the overlaid packages.

For point 2, you can do one of the following:
* Local install of PCL
  * Build and install PCL locally (eg: `$HOME/.local`) using `-DCMAKE_INSTALL_PREFIX=<install-lcoation>`
  * Configure catkin/colcon to find PCL at custom location using the following command followed by `<catkin/colcon> build`:
    * `catkin config --cmake-args -DPCL_DIR=<install-location>/share/pcl-1.<VERSION>`
    * `colcon build --packages-select-by-dep libpcl-all-dev --cmake-args "-DPCL_DIR=<install-location>/share/pcl-1.<VERSION>"`
* Use PCL in an overlay, alongside other packages dependent on PCL:
  * Clone PCL in desired workspace
  * Add a package.xml in `pcl` directory. A sample package.xml can be found [here](https://github.com/kunaltyagi/packages.xml)
  * Configure and build PCL as required
  * Build all packages

## Q: How do I build PCL with limited memory?

A: Building PCL using CMake takes a lot of space:
* All modules and features enabled (except VTK with Qt) including CUDA (Linux, x86)
  * Build: 19 GB
  * Install: 4.3 GB
* Default modules with `-DPCL_ONLY_CORE_POINT_TYPES:=ON`, VTK without Qt, CUDA (Linux, x86)
  * Build: 7.4 GB
  * Install: 2.3 GB

No documentation (html pages using doxygen/sphinx) were built in these cases.

The following instructions are untested, and would also require additional flags (since adding custom flags disables SSE/AVX and other optimizations). These are orthogonal to the above details, and as such would result in more space savings:
* For an easy approach, use
  * `-DCMAKE_BUILD_TYPE=MinSizeRel`
  * On Linux, you can also run `strip --strip-unneeded` on all binaries to remove unneeded space
* If you're ready to get your hands dirty, use `-DCMAKE_CXX_FLAGS="-Os -s -fdata-sections -ffunction-sections -Wl,--gc-sections -Wl,--strip-all"` (along with your favourite flags that you want to add. PCL builds with these flags as default: `-Wabi=11 -Wall -Wextra -Wno-unknown-pragmas -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof -Wno-conversion -march=native -msse4.2 -mfpmath=sse -fopenmp`)

If your CMake version is 3.9 or later, you can also add `-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:=TRUE` to cmake configuration. The alternative is adding `-flto` to compilation flags, but this is not supported by all compilers hence the CMake approach is recommended.

(If you can report memory usage on other platforms and configurations, please report on Discord)