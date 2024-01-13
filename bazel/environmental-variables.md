# Bazel Environmental Variables

From Codeium.

Bazel provides a set of environment variables that can be useful during test execution. Here are some commonly used Bazel environmental variables available for tests:

1. `TEST_TMPDIR`: This variable points to the temporary directory created by Bazel for the test execution. It is typically used to store temporary files or directories required for the test.
2. `TEST_SRCDIR`: This variable contains the path to the directory where the test source files are located. It can be used to access test-specific files or resources.
	1. There is a folder in here with the name of the repo, and the source directories are in there
	2. For directory structure:
	   ```
	   repo_name
	   |--- dir1
	   |--- --- dir2
	   |--- --- --- bazel_target.py
	   ```
	   the path to the source file would be something like this:
	   ```
	   /home/[username]/.cache/bazel/_bazel_[username]/5f31b1cb4b385a2fcb819b580aab3835/sandbox/linux-sandbox/50/execroot/repo_name/bazel-out/k8-opt/bin/dir1/dir2/bazel_target.runfiles/repo_name/dir1/dir2/bazel_target.py
	   ```
3. `TEST_WORKSPACE`: This variable contains the path to the root directory of the Bazel workspace. It can be used to access workspace-level files or resources.
4. `BUILD_WORKSPACE_DIRECTORY`: This variable contains the path to the directory where the BUILD file of the current target being built is located. It can be used to access files or resources relative to the BUILD file.
5. `TEST_LANGUAGES`: This variable contains the list of languages enabled for the test target. It can be used to conditionally execute or configure code based on the language.
6. `TEST_ENV`: This variable contains any additional environment variables specified for the test target. It can be used to pass custom environment variables to the tests.

These are some of the commonly used Bazel environmental variables for tests. The availability and usage of these variables may vary depending on your Bazel version and configuration. You can refer to the Bazel documentation for more details on these variables and any additional variables specific to your Bazel setup.