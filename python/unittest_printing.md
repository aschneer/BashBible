# `unittest` Library - Printing

From Codeium.

By default, the output produced by `print()` statements in Python unittests may not appear in real-time during the test execution when using Bazel. This behavior is due to the buffering of the standard output.

To ensure that the `print()` statements are displayed in real-time during the test execution in Bazel, you can disable the output buffering. Here's an example:

```python
import unittest
import sys

class MyTestCase(unittest.TestCase):
	def setUp(self):
		# Disable output buffering
		self.original_stdout = sys.stdout
		sys.stdout = sys.stderr
	
	def tearDown(self):
		# Restore output buffering
		sys.stdout = self.original_stdout
	
	def test_something(self):
		print("This message will be printed to the Bazel test output in real-time")

		self.assertEqual(2 + 2, 4, "Custom failure message that will be printed to the Bazel test output in real-time")

if __name__ == '__main__':
	unittest.main(buffer=False)
```

In this example, we override the `setUp()` and `tearDown()` methods of the `unittest.TestCase` class to disable and restore the output buffering, respectively. By redirecting `sys.stdout` to `sys.stderr` in the `setUp()` method, the output produced by `print()` statements will be immediately flushed and displayed in real-time during the test execution.

Additionally, when running the unittest using `unittest.main()`, you can pass `buffer=False` to disable the output buffering.

With these modifications, the `print()` statements will be visible in real-time during the test execution when running the tests using Bazel.

If you pass `buffer=False` to `unittest.main()`, it will disable the output buffering by default. In that case, you don't need to perform the setup and teardown steps to override `sys.stdout`.

Here's an updated example without the setup and teardown steps:

```python
import unittest

class MyTestCase(unittest.TestCase):
	def test_something(self):
		print("This message will be printed to the Bazel test output in real-time")
	
		self.assertEqual(2 + 2, 4, "Custom failure message that will be printed to the Bazel test output in real-time")

if __name__ == '__main__':
	unittest.main(buffer=False)
```

In this updated example, we remove the `setUp()` and `tearDown()` methods. Instead, we directly use `unittest.main(buffer=False)` to disable the output buffering.

By passing `buffer=False` to `unittest.main()`, the `print()` statements will be immediately flushed and displayed in real-time during the test execution when running the tests using Bazel.