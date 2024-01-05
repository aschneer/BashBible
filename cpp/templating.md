# Templating

Deducing function template type

```cpp
template <typename MyType>
void MyFunc<MyType>(const MyType& input_var) {
	std::cout << std::to_string(input_var) << std::endl;
}

// Function call with explicit type declaration:
int a = 3;
MyFunc<int>(a);

// Function call with deduced type:
	// It deduces the template type is `int` since you pass
	// in an `int` type for the templated argument.
int a = 3;
MyFunc(a);
```