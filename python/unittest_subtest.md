# unittest\_subtest

If you test multiple conditions within a single test function, rather than repeating code or writing a helper function that runs the same test code with different inputs each time, you can use the `subtest` feature.

It will loop through each test case and run the same test sequence on each one.

`unittest` will keep track of which `subTest` fails and run the others even if one fails.

```python
from my_module import divide_numbers

def test_divide_numbers(self):
    # Tuples of dividend, divisor, quotient, and assertion method.
    test_cases = [
        (4, 2, 2, self.assertEqual),
        (18, 3, 6, self.assertEqual),
        (9, 0, None, self.assertIsNone),
    ]

    for dividend, divisor, quotient, assertion_method in test_cases:
        # If mocking, you can put `mock_return=mock_object.return_value`
        # in the subTest constructor.
        with self.subTest(mock_return=mock_object.return_value):
            result = divide_numbers(dividend, divisor)
            assertion_method(result, quotient)
```

You can pass any arbitrary keyword args to `subTest()` to give context. This allows the subtests to list error messages specific to the subtest case.

```python
for dividend, divisor, quotient, assertion_method in test_cases:
    with self.subTest(dividend=dividend,
                      divisor=divisor,
                      quotient=quotient):
        result = divide_numbers(dividend, divisor)
        assertion_method(result, quotient)
```
