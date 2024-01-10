# Exceptions

Try/Except block:

```python
try:
	# Code that may raise an exception
	# ...
except ExceptionType1:
	# Code to handle ExceptionType1
	# ...
except ExceptionType2:
	# Code to handle ExceptionType2
	# ...
else:
	# Code to run if no exception occurs
	# ...
finally:
	# Code that always runs, regardless of whether an exception occurred or not
	# ...
```

Commonly used built-in exception types in Python:
- `Exception`: The base class for all built-in exceptions.
- `TypeError`: Raised when an operation or function is applied to an object of inappropriate type.
- `ValueError`: Raised when a function receives an argument of the correct type but an invalid value.
- `NameError`: Raised when a local or global name is not found.
- `IndexError`: Raised when a sequence subscript is out of range.
- `KeyError`: Raised when a dictionary key is not found.
- `FileNotFoundError`: Raised when a file or directory is requested but cannot be found.
- `IOError`: Raised when an I/O operation (such as reading or writing a file) fails.
- `ZeroDivisionError`: Raised when a division or modulo operation is performed with a divisor of zero.
- `AttributeError`: Raised when an attribute reference or assignment fails.
- `ImportError`: Raised when an import statement fails to find a module.
- `NotImplementedError`: Raised by abstract methods that should be overridden in subclasses.
- `OverflowError`: Raised when the result of an arithmetic operation is too large to be expressed in the available numeric type.
- `SyntaxError`: Raised when there is a syntax error in the code.
- `IndentationError`: Raised when there is an incorrect indentation in the code.