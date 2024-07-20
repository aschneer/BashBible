# unittest\_mock

## Import

```python
from unittest.mock import Mock, patch
```

## Usage

### Production Code

```python
def child_function(input: str):
    return input

def parent_function(input: str):
    return child_function(input)
```

### Test Code

```python
from unittest.mock import Mock, patch
from full.module.path import child_function

class MyTest(unittest.TestCase):
    @patch("full.module.path.child_function")
    def test_parent_function(self, mock_child_function):
        # When `child_function()` gets called, even indirectly,
        # the mock object will be called instead,
        # and it will return "my_string".
        mock_child_function.return_value = "my_string"
        result = parent_function()
        mock_child_function.assert_called_once()
        self.assert(result, "my_string")
        
        mock_child_function.reset_mock() # reset return value, etc.
        # Everytime `child_function()` is called,
        # even indirectly, it will raise this exception.
        mock_child_function.side_effect = ValueError("Mocked exception")
        with self.assertRaises(ValueError):
            parent_function()
        mock_child_function.assert_called_once()
```

## Patched Function Path

For the full function namespace path in `@patch()`, you need to use the namespace of the file that contains the function you're testing, not the file where the patched function is defined.

Example:

* `file_1.py` contains function `def add(a, b)`
* `file_2.py` contains function `def add_and_print()`, which calls function `add()`
* You're writing a unit test for function `add_and_print()` in a separate `test.py` file
* When you patch `add()`, you would use the namespace of `file_2.py` like:
  * `@patch("path.to.file_2.add")`
* Even though the function is defined at `path.to.file_1`, when it gets imported into `file_2.py`, it takes on the namespace of `file_2.py`
