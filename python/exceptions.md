# Exceptions

3 options:

1. `assert`
2. `raise`
3. `try`/`except` block

Try/Except block:

If you handle exceptions within `except` blocks, the program will not end. The exception will be handled, and program execution will continue. Unless you put a `raise` statement in the except block.

Python will match exceptions in the order they're listed by type in the try/except block.

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
except ExceptionType3 as e:
    # How to catch an exception, add a message to it,
    # and throw it again.
    raise ExceptionType3(f"Message about {some_variable}") from e
except (ExceptionType4, ExceptionType5, ExceptionType6):
    # How to catch multiple exceptions and apply
    # the same logic for all of them.
except Exception as e:
    # Catches any exception type that is derived from the
    # `Exception` base class.
except:
    # Catch-all - handles any exception type.
    # It will handle any exceptions leftover after
    # other except blocks. Must go at the end,
    # otherwise all exception types would trigger
    # this block first.
    # ...
else:
    # Code to run if no exception occurs
    # ...
finally:
    # Code that always runs, regardless of whether an exception occurred or not
    # ...
```

Commonly used built-in exception types in Python:

* `Exception`: The base class for all built-in exceptions.
* `TypeError`: Raised when an operation or function is applied to an object of inappropriate type.
* `ValueError`: Raised when a function receives an argument of the correct type but an invalid value.
* `NameError`: Raised when a local or global name is not found.
* `IndexError`: Raised when a sequence subscript is out of range.
* `KeyError`: Raised when a dictionary key is not found.
* `FileNotFoundError`: Raised when a file or directory is requested but cannot be found.
* `IOError`: Raised when an I/O operation (such as reading or writing a file) fails.
* `ZeroDivisionError`: Raised when a division or modulo operation is performed with a divisor of zero.
* `AttributeError`: Raised when an attribute reference or assignment fails.
* `ImportError`: Raised when an import statement fails to find a module.
* `NotImplementedError`: Raised by abstract methods that should be overridden in subclasses.
* `OverflowError`: Raised when the result of an arithmetic operation is too large to be expressed in the available numeric type.
* `SyntaxError`: Raised when there is a syntax error in the code.
* `IndentationError`: Raised when there is an incorrect indentation in the code.

From Bard:

**Here's a guide to when to use `assert`, `raise`, and `try` in Python:**

**Assert:**

* **Purpose:** To verify internal program logic and assumptions during development and testing.
* **Behavior:** If the assertion fails, an `AssertionError` is raised, typically halting execution.
* **Use cases:**
  * Checking preconditions for functions and methods.
  * Validating data invariants within functions and classes.
  * Ensuring expected states during testing.
* **Example:**

Python

```python
def calculate_area(width, height):
    assert width > 0 and height > 0, "Dimensions must be positive"
    return width * height
```

Use code with caution. [Learn more](https://bard.google.com/faq#coding)

**Raise:**

* **Purpose:** To signal explicit errors or exceptional situations that require handling.
* **Behavior:** Raises a specified exception, which can be caught using `try-except` blocks.
* **Use cases:**
  * Custom errors for specific failure conditions.
  * Signaling errors from external sources (e.g., file I/O, network operations).
  * Re-raising caught exceptions for further handling.
* **Example:**

Python

```python
class InvalidInputError(Exception):
    pass

def divide(x, y):
    if y == 0:
        raise InvalidInputError("Cannot divide by zero")
    return x / y
```

Use code with caution. [Learn more](https://bard.google.com/faq#coding)

**Try:**

* **Purpose:** To handle potential exceptions and control program flow when errors occur.
* **Behavior:** Executes code within the `try` block, catching specified exceptions in `except` blocks.
* **Use cases:**
  * Preventing program crashes from unexpected errors.
  * Implementing alternative actions or error recovery.
  * Cleaning up resources (e.g., closing files) even when errors occur.
* **Example:**

Python

```python
try:
    with open("data.txt", "r") as file:
        data = file.read()
except FileNotFoundError:
    print("File not found")
```

Use code with caution. [Learn more](https://bard.google.com/faq#coding)

**Key Considerations:**

* **Assertions:** Primarily for debugging and testing, not for runtime error handling.
* **Raising Exceptions:** Used to signal errors that need to be addressed.
* **Try-Except:** Catches and handles exceptions for controlled error management.

Choose the appropriate construct based on your specific needs for debugging, error signaling, or error handling in different scenarios.
