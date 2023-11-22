# Function Objects

Function objects are used for:
- Storing a function to a variable
- Creating a pointer to a function
- Callback function - Passing a function as an argument to another function

```cpp
#include <functional>
std::function
```

Function pointers - Google Bard example:

```cpp
void modifyFunction(void (*funcPtr)(int)) {
	// Modify the function pointed to by funcPtr
	funcPtr = &anotherFunction;
}

void originalFunction(int x) {
	std::cout << "Original function: x = " << x << std::endl;
}

void anotherFunction(int x) {
	std::cout << "Modified function: x = " << x << std::endl;
}

int main() {
	// Declare a function pointer
	void (*funcPtr)(int) = &originalFunction;

	// Call the modifyFunction function
	modifyFunction(funcPtr);

	// Call the function pointer
	funcPtr(5);
	
	return 0;
}
```

Function pointers

```cpp
void originalFunction(int x) {
	std::cout << "Original function: x = " << x << std::endl;
}

// Declare a function pointer, and assign the
// existing function to it.

void (*funcPtr)(int); // Assign later
funcPtr = &originalFunction;

void (*funcPtr)(int) = &originalFunction; // Assign now.

// Multiple arguments
void (*funcPtr)(int, std::string);

// Call a function by its function pointer
(*funcPtr)(42);
funcPtr(42); // This also works, but less clear what you're doing.

// THE BELOW MIGHT ALSO WORK, NOT SURE.

// Passing by function pointer, defined beforehand.
typedef return_type (*func_ptr)(arg1_type, arg2_type);
void my_func (func_ptr func_name) {}

// Passing by reference
void my_func (return_type (&func_name)(arg1_type, arg2_type)) {}
```

Creating a variable to store a function. This is different from function pointers.

```cpp
std::function<return_type(arg1_type, arg2_type)> func_name;
```

Lambdas:

These are defined in place and temporary by nature.

```cpp
// Syntax
[capture_list](parameter_list) -> return_type { function_body }
	// capture_list = parameters from surrounding scope
	//	that the lambda function body needs to access.
	// parameter_list = arguments to the lambda function
	// return_type = type of function return
	// function_body = body of lambda function, each line
	//	ending in semicolon.

// Accept a lambda as input to a function
void func(std::function<return_type(arg_type)> lambda_func) {
	return_type my_var;
	my_var = visitor(arg_type);
}
// Example
void func(std::function<std::string(int)> lambda_func) {
	std::string my_string;
	my_string = lambda_func(6);
}
```

Not sure how to use this:

```cpp
// Save a lambda to a variable
std::function<return_type(arg1_type, arg2_type)> func_name;
```