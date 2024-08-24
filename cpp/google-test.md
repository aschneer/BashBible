# google-test

## Test that Function Call Succeeds

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

## Test that Function Call Causes Death

How to test that a `CHECK` or exception does crash the function under test:

```cpp
EXPECT_DEATH(
    FunctionUnderTest(arg1, arg2),
    ".*" // regex looks for specific stdout message; ".*" accepts any.
) << "Test fail message";
```

## Mocking

In C++, there is no way to mock a class without setting up the actual code to support mocking.

The basic idea is to make a derived class of the class you want to mock, and override all the methods you want to mock.

The problem is all the methods to mock must be virtual in the base class. If they are not, the code should be modified as follows.

In the actual code:

1. Create an abstract interface class consisting of only virtual functions
   1. This class defines the interface that the concrete class (to mock) will have
   2. Interface class uses same member/method names that derived concrete classes will use
2. The concrete class (to mock) inherits from the interface class
   1. Override interface methods as needed
3. Anywhere in the code where the concrete class is to be mocked, make sure the abstract interface class is used as the type
   1. In the production code, all types used for function arguments and returns should be the abstract interface class type. Instances of the concrete class can be stored and passed through these types.
   2. In the production code, anywhere where an actual instance is constructed, it must be the concrete class. You cannot create an instance of a pure abstract class.
      1. If you declare a variable to hold the class but don't initialize it, use the abstract interface.
      2. Anytime you initialize, you need to use the concrete class.
      3. Once you initialize, you can then store the concrete instance to the abstract class type.
      4. If you intend to have a pointer to the class, declare the pointer as the abstract class type, and when you allocate an instance, make the instance the concrete class then store it to the pointer.
   3. For example, if there is a function that takes the concrete class as an argument, and you want to be able to pass a mock version to that argument, change the argument type to the abstract interface class.
   4. Since the concrete class and mock class both inherit from the abstract interface base class, you must use the base class type anywhere you want both the concrete class and mock class to be accepted, otherwise it will throw a type mismatch error.

In the test code:

1. Create a mock class that also inherits from the interface class in the real code
2. Override all the functions you want to mock in the mock class
   1. You can use `MOCK_METHOD()` from google mock, but I haven't gotten this to work.
3. Instantiate the mock class and use it in place of the real class in the test

The reason you need a common interface class is you need the types to match.

If you were to just create a dummy class and try to pass it as a mock object to a function that is expecting the real object, there would be a type mismatch because the mock class has no polymorphic relationship to the class being mocked (which the function expects).

### Can you Override Just the Constructor?

What if all the methods of the base class are only called by its constructor? In this case, if you wanted to block the class completely in a mock, you could theoretically just mock the constructor.

In C++, constructors are not declared as `virtual` because they are not overridden per se, rather every derived class and base class has its own constructor. Derived classes have to be constructed on their own explicitly.

So by this logic, if you just create a derived mock class from the base class and define a constructor in the mock class, you would block everything in the base class, right?

Wrong. In C++, when a derived class is instantiated, the base class constructor is automatically run first before the derived class constructor. There's no way to prevent this other than doing the interface class strategy described above.

### Mock Method

```cpp
#include "gtest/gtest.h"
#include "gmock/gmock.h"

MOCK_METHOD()
```

{% hint style="info" %}
When using `MOCK_METHOD()`, beware of commas. This macro does not like commas within one of the arguments. If there are commas, you must wrap the section with the comma in parentheses.

This often happens when you include a complex data type which has a double template.

For example:

```cpp
const std::vector<std::pair<int, std::string>>
```

You would have to wrap this whole thing in parentheses.

```cpp
(const std::vector<std::pair<int, std::string>>)
```
{% endhint %}

### Mock Asserts

```cpp
#include "gtest/gtest.h"
#include "gmock/gmock.h"

ON_CALL()
EXPECT_CALL()
```

### Mock Matchers

```cpp
#include "gtest/gtest.h"
#include "gmock/gmock-matchers.h"

EXPECT_THAT()
```
