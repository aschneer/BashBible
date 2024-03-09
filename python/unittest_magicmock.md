From Codeium.

The `MagicMock` class in Python's `unittest.mock` module provides a wide range of capabilities for creating and configuring mock objects. Here are some of the key features and methods of the `MagicMock` class:

1. **Return Values**:
    - `return_value`: Set the return value of the mock.
    - `side_effect`: Specify a function or exception to be called or raised when the mock is called.
2. **Call Handling**:
    - `assert_called_once()`, `assert_called_with()`: Assert that the mock was called a certain number of times with specific arguments.
    - `call_args`, `call_args_list`: Retrieve the arguments with which the mock was called.
    - `assert_called_once_with()`, `assert_called_with()`: Assert that the mock was called with specific arguments.
3. **Behavior Control**:
    - `reset_mock()`: Reset the mock's state, including call history and return value.
    - `attach_mock()`, `detach_mock()`: Attach or detach another mock object.
4. **Attribute Access**:
    - Allow access to attributes like `called`, `call_count`, `return_value`, `side_effect`, etc.
5. **Magic Methods**:
    - Allow customization of behavior for special methods (e.g., `__getitem__`, `__iter__`, `__enter__`, `__exit__`, etc.).
6. **Property Mocking**:
    - `property.mock`: Create a mock for a property.

These are just a few examples of the features and methods provided by the `MagicMock` class. The full range of capabilities and methods can be found in the official Python documentation for the `unittest.mock` module.

---

From Gemini.

MagicMock is a powerful mocking object included in the `unittest.mock` library of Python. It provides a flexible way to simulate the behavior of real objects during testing. Here's a breakdown of its key functionalities:

**Attributes:**
- **`mock.called` (bool):** Indicates if the mock has been called at least once.
- **`mock.call_args` (tuple):** Contains arguments passed during the last call.
- **`mock.call_args_list` (list):** Stores arguments passed during all calls.
- **`mock.return_value` (object):** Sets the value to return when the mock is called.
- **`mock.side_effect` (callable):** Defines a function to execute as a side effect when the mock is called.

**Methods:**
- **`mock.assert_called()`:** Raises an assertion error if the mock hasn't been called.
- **`mock.assert_called_once()`:** Raises an assertion error if the mock hasn't been called exactly once.
- **`mock.assert_called_with(*args, **kwargs)`:** Raises an assertion error if the mock wasn't called with the specified arguments.
- **`mock.assert_any_call(*args, **kwargs)`:** Raises an assertion error if none of the calls match the arguments.
- **`mock.assert_has_calls(calls, any_order=False)`:** Asserts that specific calls were made (optionally in any order).
- **`mock.reset_mock()`:** Resets the mock's state, clearing call history and side effects.

**Mocking Attributes:**
- **`mock.method_name.return_value = value`:** Sets the return value for a specific method of the mock.
- **`mock.method_name.side_effect = function`:** Sets a side effect function for a specific method.

**Context Managers:**
- **`with mock() as m:`:** Creates a temporary mock object and assigns it to a variable within the `with` block. This automatically resets the mock at the end of the block.

**Example:**

```python
from unittest.mock import MagicMock

def my_function(arg1, arg2):
  # Do something with arguments
  return arg1 + arg2

# Mock the function
mock_func = MagicMock(return_value=10)

# Call the mock
result = my_function(5, 3)

# Assertions
mock_func.assert_called_once_with(5, 3)
assert result == 10

# Set a side effect
mock_func.side_effect = lambda a, b: a * b

# Call again with different behavior
result = my_function(2, 4)
assert result == 8
```

This is just a brief overview. The `unittest.mock` documentation provides comprehensive details on MagicMock's capabilities and various other mocking features: [https://docs.python.org/3/library/unittest.mock.html](https://docs.python.org/3/library/unittest.mock.html)