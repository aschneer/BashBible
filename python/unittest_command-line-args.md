**`unittest` Command Line Arguments**

Table of Contents:

```table-of-contents
style: nestedList # TOC style (nestedList|inlineFirstLevel)
minLevel: 0 # Include headings from the specified level
maxLevel: 0 # Include headings up to the specified level
includeLinks: true # Make headings clickable
debugInConsole: false # Print debug info in Obsidian console
```

# Method 1

Let's say you have a python module like this:

```python
import sys
import unittest

class MyTestClass(unittest.TestCase)
	def setUp(self)
		# stuff
	def test_test1(self)
		# stuff
		# assert...

if __name__ == '__main__':
	unittest.main()
```

When `unittest.main` runs, it will try to parse any comment line arguments passed to this Python module. `unittest` supports its own flags, so it's looking for things like `-v` for verbosity, etc.

If you don't pass anything to `unittest.main()`, it will parse the arguments that are in `sys.argv`. `sys.argv` consists of the first element `sys.argv[0]` which is the name of the python module, followed by as many command line arguments as there are (`sys.argv[1:]`).

If there are arguments in `sys.argv` that `unittest` doesn't recognize, it will give an "unrecognized arguments" error.

So to prevent this, you can pass an empty set of args to `unittest.main` using the `argv=` keyword argument, and it will parse that instead of parsing `sys.argv`. You just have to make sure that the other list's first element is the name of the Python module.

There will be no unrecognized args since the list will be empty.

This would look like:

```python
if __name__ == '__main__':
	# Grab module name from first element of sys.argv
	unittest.main(argv=[sys.argv[0]])
```

The downside of this is no command line arguments will be passed to `unittest.main`, and you might want to pass some, like `-v` for verbosity.

`unittest` doesn't support custom command line arguments, so you might also want to use an argument parser like `argparse` to define custom arguments for your module and use them in your test. In that case, you will want to separate your custom arguments for the test from other command line arguments intended for `unittest` (like `-v`).

This can be accomplished by defining your custom arguments with `argparse` and then using `parse_known_args()` to extract the known arguments (the ones you defined) separately from the unknown arguments.

```python
import argparse
import sys
import unittest

class MyTestClass(unittest.TestCase)
	def setUp(self)
		# stuff
	def test_test1(self)
		# stuff
		# assert...

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--arg1', required=True)
	
	# First return is recognized args, second is unrecognized.
	#
	# Define which args to parse by passing the sys.argv
	# list excluding the first element. This is done by default,
	# by we're doing it explicitly here for clarity.
	args, unittest_args = parser.parse_known_args(sys.argv[1:])
	
	# Now we pass all the "unrecognized" args to unittest.
	#
	# Make sure to pass the module name as the first element
	# of the argument list.
	unittest.main(argv = [sys.argv[0], *unittest_args])

	# This is an equivalent way to write the same line.
	unittest.main(argv = [sys.argv[0]] + unittest_args)
```

Keep in mind, `unittest_args` can't have a bunch of junk in it. It can only have arguments that `unittest` will recognize. This method is intended to allow passing of one group of arguments to your test code, and another group to the `unittest` base class itself.

---

The following is another way to do the same thing. But I prefer the previous method because you don't have to pass the same flag a bunch of times, you can just pass the arguments you want to pass and they will go where they need to go.

```python
if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--arg1', required=True)
	
	# Define an argument that can be used any number of
	# times, and each time its value will be added to a list.
	# In the end, `args.extra_cmd_line_args` will contain
	# a list of all the values passed to all occurrences of
	# the flag.
	parser.add_argument("--extra_cmd_line_args",
						action="append", default=[sys.argv[0]])
	args = parser.parse_args()
	# The `args` namespace object seems to automatically
	# add the module name to the front of the arg list
	# when it is used, so we don't have to do that here.
	unittest.main(argv=args.extra_cmd_line_args)
```

# Method 2

You can also pass arguments to a `unittest` via environmental variables. Bazel test rules such as `py_test` support these as follows:

```python
py_test(
	# ...
	# ...
	env = {
		"MY_VAR_1": "35",
		"MY_VAR_2": "hello",
	}
)
```

In the python test, you would use `os.environ.get("MY_VAR_1")` to get the values.