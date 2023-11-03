# Bazel Reference

```bash
-c opt # C-optimized build (faster)

# Build entire project from root directory.
bazel build -c opt //...

# Build a package called "arm_motion" within the directory
# //my_project/robot/arms.
bazel build -c opt //my_project/robot/arms:arm_motion

# Run a package called "arm_motion" within the directory
# //my_project/robot/arms. If it's a c++ package, it will
# run the main function from arm_motion.cc (or .cpp).
bazel run -c opt //my_project/robot/arms:arm_motion

# Check your change.
bazel run //:buildifier

bazel run -c opt :log_extractor -- --ingest_tool_release

# Edit files in place, fix formatting issues.
bazel run //:buildifier
bazel run //:clang_format

# Run a test in the folder containing the test.
bazel test -c opt ...
```