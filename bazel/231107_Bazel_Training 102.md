# 231107 Bazel Training 102

Alex Eagle from Aspect.

## Bringing a code base into Bazel

### How to bring external CMake project into a Bazel project

- Can definitely do it
- Wrap it in Bazel and Bazel will still execute the CMake build

## Profiles

Every Bazel build generates a profile with data on results of build speed. Get the location by running `bazel info output_base`.

## Watch Mode

[bazel-watcher](https://github.com/bazelbuild/bazel-watcher)

`ibazel`

Download binary and add to `$PATH`

## Custom Rules

