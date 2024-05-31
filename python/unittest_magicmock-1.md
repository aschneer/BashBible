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

<pre class="language-python"><code class="lang-python">from unittest.mock import Mock, patch
from full.module.path import child_function
<strong>
</strong><strong>class MyTest(unittest.TestCase):
</strong>    @patch("full.module.path.child_function")
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
</code></pre>
