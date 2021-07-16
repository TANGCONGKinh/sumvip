# Ideas for GSoC/GSoD

PCL returns to GSoC after a long hiatus. This year, PCL is looking for contributions and active community members from all corners of the coding community in the roles of
* technical writers
* designers
* web developers
* C++ developers
* CI/CD experts
* polyglots
* wizards

and everything else in between.

## 2021 GSoC Idea List
2021 is a special year in the sense that the projects need to be much smaller. This potentially allows "easier" projects to be curated by the projects. Moreover, the reduced time requirement makes GSoC accessible to more students.
### Table of Contents
New Ideas
| Name | Skills needed | Difficulty |
|---|---|---|
| [Enable CUDA builds on CI](#Enable-CUDA-builds-on-CI) | CUDA, C++, CMake | Medium |
| [Better Voxel Filter](#Better-Voxel-Filter) | C++ | Low|
| [Bindings for Python](#Bindings-for-Python) | C++, python, clang | Medium |
| [Shift to fixed-width integers](#Shift-to-fixed-width-integers) | C++ | Low|
| [Benchmarks and Performance monitoring](#Benchmarks-and-Performance-monitoring) | C++, Benchmarking, Github Worfklow | Medium-Low |

Older ideas under progress:
| Name | Skills needed | Difficulty |
|---|---|---|
| [Compilation time reduction](#Compilation-time-reduction) | C++, C++ compilation, CMake | High |
| [Improving confidence in builds](#Improving-confidence-in-builds) | Bash, Static Analyzers, GitHub Workflow | Low |
| [Make a better CMake](#Make-a-better-CMake) | CMake | Medium |
| [Refactoring and Modernization](#Refactoring-and-Modernization) | C++ | Low |
| [Unified API for PCL algorithms](#Unified-API-for-PCL-algorithms) | C++ | Medium-High |

## 2021 GSoD Idea List

### Table of Contents
| Name | Skills needed | Difficulty |
|---|---|---| 
| [Update outdated Website](#Update-outdated-Website) | RST, Sphinx, Doxygen, Python 	 | Medium |
| [Walk through documentation](#Walk-through-documentation) | Technical writing, Doxygen | Medium |
| [Reference Manual Documentation](#Reference-Manual-Documentation) | Technical writing, Doxygen, Sphinx | Low |

## 2020 GSoC Idea List

### Table of Contents
| Name | Skills needed | Difficulty |
|---|---|---| 
| [Benchmarks and Performance monitoring](#Benchmarks-and-Performance-monitoring) | C++, Benchmarking | Medium-Low |
| [Compilation time reduction](#Compilation-time-reduction) | C++, C++ compilation, CMake | High |
| [Improving confidence in builds](#Improving-confidence-in-builds) | Bash, Static Analyzers | Low |
| [Make a better CMake](#Make-a-better-CMake) | CMake | Medium |
| [Refactoring and Modernization](#Refactoring-and-Modernization) | C++ | Low |
| [Binding interfaces for other languages](#Binding-interfaces) | C++, libtooling, pybind/cython,etc. | High |
| [Unified API for PCL algorithms](#Unified-API-for-PCL-algorithms) | C++ | Medium-High |

## 2020 GSoD Idea List

### Table of Contents
| Name | Skills needed | Difficulty |
|---|---|---| 
| [Update outdated Website](#Update-outdated-Website) | RST, Sphinx, Doxygen, Python 	 | Medium |
| [Walk through documentation](#Walk-through-documentation) | Technical writing, Doxygen | Medium |
| [Reference Manual Documentation](#Reference-Manual-Documentation) | Technical writing, Doxygen, Sphinx | Low |

### Project Maintenance (`proj`)
As the community grows, soft-skills and community outreach become more important. With the change in community, it has become important to update the overlooked facet of PCL and make it more accessible to the community
#### Update outdated Website
  * Create a static website on Github pages or similar
  * Provide a basic replacement including:
    * documentation for past releases
    * prevent invalidation of links as much as possible, while updating the content
  * Make it the new home for `pointclouds.org` with HTTPS support
  * Integrate versioned (documentation+tutorials) to make it the go-to resource for PCL
  * (Stretch goal) Migrate all content from existing website, including 
    * ability to create blog posts (using Jekyll or similar)
    * lost content (from internet archives)
#### Walk through documentation
[Summer of Docs focus] Includes Tutorials, Guides and Examples
  * Update to reflect forward movement in PCL
  * Detect and increase coverage
#### Reference Manual Documentation
  * [Summer of Docs focus] Improve documentation coverage
  * Migrate front-end to sphinx
  * Generate and host all documentation on Github
  * Integrate search + landing page with Tutorials and Walk-through
  * (Stretch goal) Add CI test for missing documentation in "new code"

### Quality of Life (`QoL`)
As PCL has matured, it has discovered missing features: features that are more than just the core code, features that are vital for delivering continuous improvements. We look forwards for the ideas/projects with a minor or negligible addition of "code features" as output. Some of the wish-list features are:
#### Benchmarks and Performance monitoring
**Mvieth** has started the work of adding benchmarks that can be run offline. The aim is two fold:
  1. Increase coverage of benchmarks
  2. Better integration of benchmarks

While increasing coverage implies adding more (and better) benchmarks, a better integration would involve:
  * Nightly CI jobs to measure performance on `master`
  * (Stretch goal) Incremental bench-marking on PR
#### Compilation time reduction
  * Refactoring/CMake changes/Sub-Modules to reduce compilation time
  * 100% reproducible incremental builds (with a constant config)
  * (Stretch goal) Compile time reduction on CI using ([reading resource](https://onqtam.com/programming/2019-12-20-pch-unity-cmake-3-16/))
    * incremental builds
    * compiler caches
    * PCH
    * Unity builds
#### Improving confidence in builds
This is a wide ranging topic, with something for everyone. This is broken into bite-sized projects, each with a unique challenge.
  * Incremental builds on CI (Github Workflow)
  * More warnings and sanitizers for CI (CMake, Compilers)
  * Integrate `clang-tidy`, static-analyzers (C++ tooling, easier for new-comers)
  * Increasing test coverage (C++)
  * ABI/API breakage monitoring for PR (command line tools, scripting)
#### Make a better CMake
  * More DRY, less wizardry
  * Automatic module discovery
  * Automatic test discovery (refactoring test code layout is ok)
  * Out-of-source PCL-contrib super-module similar to OpenCV-contrib
#### Enable CUDA builds on CI
A big issue with the CUDA and GPU libraries in PCL is the inability to test them on the CI. Thankfully, AMD (yes, you read that correct) has a solution: HiP. HiP compiler allows compilation of CUDA code to run on either AMD GPUs or even on the CPU. Thus, a test for CUDA can be compiled and run on the free tiers of CI which don't offer any GPU :)

For more information, please refer to:
* https://rocmdocs.amd.com/en/latest/Programming_Guides/Programming-Guides.html
* https://rocmdocs.amd.com/en/latest/Programming_Guides/HIP-porting-guide.html

Some repositories related to HiP are:
* https://github.com/ROCm-Developer-Tools/HIPIFY (convert CUDA code to HiP code)
* https://github.com/ROCm-Developer-Tools/HIP
* https://github.com/ROCm-Developer-Tools/HIP-CPU
* https://github.com/ROCm-Developer-Tools/HIP-Examples

### Code Maintenance (`code`)
Old API designs result in greater friction between how developers prefer to use vs how PCL lets developers use the API. Overhauling API used to be difficult but with `libtooling` and `clang-tidy`, it has been made manageable. Some of the ideas for guiding PCL towards a more modern API are:
#### Refactoring and Modernization
  * Fluent style API for algorithms
  * Extend `format` and `clang-tidy --fix` to other modules
#### Shift to fixed-width integers
PCL used integer types like `int`, which make it hard to present a uniform interface on multiple platforms. As such, we're migrating slowly to fixed width types, with the first wave focused on the type of the indices used by PCL.
  * [Better type for indices](https://github.com/PointCloudLibrary/pcl/wiki/PCL-RFC-0002:-Better-type-for-indices)
  * [CI tracking the progress](https://github.com/PointCloudLibrary/pcl/blob/master/.ci/azure-pipelines/build/ubuntu_indices.yaml)
#### Better Voxel Filter
There are a number of voxel filters in PCL, with very similar code. Analysis of the code reveals that there are 3 orthogonal decisions taken in each implementation: 
* Sub-divide the space into voxels (spherical, cubic)
* Merge points (average, centroid, etc.)
* Filter points based on some statistic (minimum count, stddev, etc.)

A well written single implementation would be able to cover all the current classes, reduce bugs due to divergence of code as well as allow easy updates like parallelism and vectorization.

### New features (`new`)
PCL is missing a lot of state-of-the-art implementations. Suggestions/implementations of such algorithms are highly welcome since highly performant, cutting edge algorithms are a core component of PCL. Apart from them, the following features are also on the wish-list of PCL:
#### Binding interfaces
Wrappers for (Python, Matlab/Octave, C) (1 project per wrapping isn't worth 3 months)
  * Needs auto-discovery (like [binder](https://github.com/RosettaCommons/binder))
    * Reduce drift between core code and bindings
    * Reduce maintenance burden
  * Tests
  * Existing wrappers for Python for inspiration
    * [pcl-py](https://github.com/strawlab/python-pcl)
    * [pypcl](https://github.com/davidcaron/pclpy)
  * (Stretch goal) Support different PCL releases and Language versions
#### Unified API for PCL algorithms
Please find a detailed document [here](https://github.com/PointCloudLibrary/pcl/wiki/PCL-RFC-0003:-Unified-API-for-Algorithms)
#### Bindings for Python
This is a continuation of the 2020 GSoC. The 2020 GSoC involved a very deep dive into different strategies for auto-generating bindings with minimal effort. This would involve making the tooling better. While the [existing code](https://github.com/PointCloudLibrary/clang-bind) can't generate bindings, the goal is almost within reach. To generate bindings automatically, new development would need to be focused on:
  * Generate list of files to read using `compile-commands.json`
  * Generate dependency (for compilation) using [CMake File API](https://cmake.org/cmake/help/latest/manual/cmake-file-api.7.html)
  * Convert Clang's AST into a Pybind11 compatible code

## Contact
Please read [the relevant official guides](https://developers.google.com/open-source/gsoc/resources/guide) to ease your journey.

For interested students:
* Please announce yourself and your interest in PCL at the [discord channel](https://discord.gg/JFFMAXS) 
* Please have a draft proposal ready in the format of a git-repo/google doc to facilitate discussion
* Based on your interests, you will be assigned a small task to evaluate your skills

For those interested in becoming mentors:
* Please note that mentoring and co-mentoring can take a lot of effort from your side (from 10 hours a week or 1 hour a week depending on the project, the student, the problems, etc.)
* Please announce yourself and participate in discussions at the [discord channel](https://discord.gg/JFFMAXS)
* Feel free to PM existing maintainers/mentors
* "Teaming up" as co-mentors is an option if you feel you will not be able to be the sole mentor on a project. For projects with existing mentor(s), it is upto them to accept/reject a co-mentor

### Mentors
Alphabetical list of mentors
* kunaltyagi
* SergioRAgostinho
* taketwo
and others

## Resources
### Ideas Pages
If any of the following ideas are not implemented, please notify [kunaltyagi](https://github.com/kunaltyagi) on the [discord channel](https://discord.gg/JFFMAXS). They will be added to the ideas list.
* [Discussion on Ideas for GSoC 2011](http://www.pcl-developers.org/two-more-projects-for-GSOC-tt4645184.html#none)
* [Ideas page for GSoC 2011](https://web.archive.org/web/20130314145536/http://www.pointclouds.org:80/gsoc2011/ideas.html)
<!-- * [Ideas page for GSoC 2014](http://www.pointclouds.org/gsoc/) State of the art is Deep Learning -->
### Project Pages
If any of the following projects are not integrated in PCL, please notify [kunaltyagi](https://github.com/kunaltyagi) on [discord channel](https://discord.gg/JFFMAXS). They will be slated for review and addition in PCL-core (if maintainers are found) or PCL contrib (like opencv-contrib) (if there is a lack of maintainers)
* [GSoC 2011](http://www.pointclouds.org/blog/gsoc/)
* [GSoC 2012](https://web.archive.org/web/20121009031358/http://pointclouds.org:80/news/pcl-gsoc-kickstart.html)
* [GSoC 2014](http://www.pointclouds.org/blog/gsoc14/index.php)