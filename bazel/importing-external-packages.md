# Importing Non-Bazel External Packages into a Bazel Project

To do this, you need to wrap the external library in a Bazel target (either a `cc_binary` or `cc_library` rule).

## Build File

Create a BUILD.xxx.bazel file where "xxx" is the name of the external library (for reference purposes). The file should contain a `cc_binary()` rule for the wrapped target. This file can go anywhere in the Bazel project, for example in an "external_libraries" directory.

This is the Bazel target that wraps the imported library that is non-bazel.

```python
# Filename = BUILD.ext_package_name.bazel

package(default_visibility = ["//visibility:public"])

cc_library(
	# This is the name of the Bazel target that is
	# wrapping the external library. This is the name
	# you will use to list this external package as a
	# dependency in cc_library and cc_binary rules
	# in BUILD.bazel files.
	name = "my_wrapper_target_name",
	# This will search through the external library
	# directory recursively and add as a source any
	# .cpp files it finds.
	srcs = glob([
		"src/**/*.cpp",
	]),
	# Path to the header files within the
	# external library directory, starting at root.
	# This syntax will return all .h files found
	# recursively from "headers" downward.
	hdrs = glob(["include/headers/**/*.h"]),
	# This will strip a portion of the path within
	# the external library directory when you refer
	# to its files from the Bazel project. For example,
	# if you're including a header from the external library
	# in a c++ source file,
	# instead of saying `#include "include/headers/header_file.h"`
	# you can just say `#include "header_file.h"`.
	strip_include_prefix = "include/headers/",
)
```

As a concrete example, here's what it would look like for the [urdfdom](https://github.com/ros/urdfdom) library (part of ROS):

```python
# Filename = BUILD.urdfdom.bazel

package(default_visibility = ["//visibility:public"])

cc_library(
	name = "urdf_parser",
	srcs = glob([
		"urdf_parser/src/**/*.cpp",
	]),
	# This is the actual path to the header in the urdfdom
	# directory. The first "urdf_parser" folder sits in
	# the root directory.
	hdrs = [
		"urdf_parser/include/urdf_parser/urdf_parser.h"
		"urdf_parser/include/urdf_parser/exportdecl.h"
	],
	strip_include_prefix = "urdf_parser/include/",
)
```

## WORKSPACE File

Put an `http_archive()` rule in the `WORKSPACE` file, which is in the root directory of the Bazel project, and defines the Bazel project itself. Or, put the rule in a file that `WORKSPACE` loads (imports).

Example:

```python
# Filename = WORKSPACE

http_archive(
	name = "my_http_archive_target_name",
	build_file = "@bazel_root//path/to/BUILD.ext_package_name.bazel",
	sha256 = "sha256_of_release_tarball_which_can_get_from_curl",
	strip_prefix = "root_directory_name_of_tarball_after_extracting",
	urls = ["url_to_tarball_of_release_on_github"],
)
```

Concrete example with [urdfdom](https://github.com/ros/urdfdom):

```python
# Filename = WORKSPACE

http_archive(
	name = "urdfdom",
	build_file = "@bazel_root//external_libraries:BUILD.urdfdom.bazel",
	sha256 = "3c780132d9a0331eb2116ea5dac6fa53ad2af86cb09f37258c34febf526d52b4",
	strip_prefix = "urdfdom-3.0.0",
	urls = ["https://github.com/ros/urdfdom/archive/refs/tags/3.0.0.tar.gz"],
)
```

## Using the External Library

### BUILD.bazel

First, add a Bazel `cc_binary` or `cc_library` rule to the `BUILD.bazel` file in the directory where the new source file will live.

```python
# Filename = BUILD.bazel

cc_binary(
	name = "my_app",
	srcs = ["my_app.cpp"],
    visibility = ["//visibility:public"],
    deps = [
	    "@my_http_archive_target_name//:my_wrapper_target_name"
    ],
)
```

Concrete example with [urdfdom](https://github.com/ros/urdfdom):

```python
# Filename = BUILD.bazel

cc_binary(
	name = "my_app",
	srcs = ["my_app.cpp"],
	visibility = ["//visibility:public"],
	deps = [
		"@urdfdom//:urdf_parser"
	],
)
```

### New App Source File

Include it in a new C++ source file:

```cpp
#include "urdf_parser/urdf_parser.h"
```