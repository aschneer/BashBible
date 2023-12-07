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

# Run a specific module with a flag.
bazel run -c opt :my_module -- --my_flag

# Edit files in place, fix formatting issues.
bazel run //:buildifier
bazel run //:clang_format

# Run all tests in the current folder.
cd folder_containing_test
bazel test -c opt ...
# Run all tests in any folder.
bazel test -c opt //path/to/my/folder/...
# Useful options for `bazel test`:
	# Show full output, expected vs. result
	--test_output=streamed
	# Rerun every test, don't use cached results from a previous run.
	--nocache_test_results
	# Print expected and actual results for all tests.
	--test_output=all
	# Print full expected output for any test that times out.
	--test_verbose_timeout_warnings
	# Print full expected output for all tests that fail.
	--test_output=errors

# Print locations of everything
bazel info

bazel fetch
```