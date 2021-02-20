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
