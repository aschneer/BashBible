# google-test

How to test that a `CHECK` does not crash the function under test:

```cpp
EXPECT_EXIT(
    {
        FunctionUnderTest(arg1, arg2);
        ::exit(0);
    },
    ::testing::ExitedWithCode(0),
    ".*" // regex looks for specific stdout message; ".*" accepts any.
);
```

How to test that a `CHECK` or exception does crash the function under test:

```cpp
EXPECT_DEATH(
    FunctionUnderTest(arg1, arg2),
    ".*" // regex looks for specific stdout message; ".*" accepts any.
) << "Test fail message";
```
