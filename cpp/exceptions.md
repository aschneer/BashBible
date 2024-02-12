# Exception Handling

From Gemini (Bard) (mixed).

```table-of-contents
style: nestedList # TOC style (nestedList|inlineFirstLevel)
minLevel: 0 # Include headings from the specified level
maxLevel: 0 # Include headings up to the specified level
includeLinks: true # Make headings clickable
debugInConsole: false # Print debug info in Obsidian console
```

## Summary

3 Options:
1. `assert`
2. `throw`
3. `try`/`catch` block

### Assertions

```cpp
#include <assert.h> // C-style assert, less functionality, less preferred
#include <cassert> // C++ style, more functionality, recommended

assert(b != 0); // No error message
assert(value > 0 && "Value must be positive"); // C++ only

// If assertion fails, it throws type...
AssertionError
```

### Throw

```cpp
throw std::error_type("Error message..."); // throw is a built-in keyword
```

### Try/Except Blocks

#### Syntax

```cpp
#include <iostream>
#include <stdexcept>

try {
	// Code that might throw an exception
	throw // throw manually if code will not throw on its own.
} catch (exception_type1 exception_name) {
	// Handle exception of type exception_type1
} catch (exception_type2 exception_name) {
	// Handle exception of type exception_type2
} catch (const std::exception& e) {
	// The last `catch` block can be the generic
	// exception type to handle any unexpected ones.
	std::cerr << "Unexpected error: " << e.what() << std::endl;
}
// ... more catch blocks for other exceptions
```

#### Examples

```cpp
#include <iostream>
#include <stdexcept>

try {
	readFile("data.txt");
} catch (const std::exception& e) {
	std::cerr << "Error: " << e.what() << std::endl;
}
```

```cpp
#include <iostream>
#include <stdexcept>

int main() {
	int age;
	
	try {
		std::cout << "Enter your age: ";
		std::cin >> age;
		if (age < 0) {
			throw std::invalid_argument("Age cannot be negative.");
		}
	} catch (const std::invalid_argument& e) {
		std::cerr << "Error: " << e.what() << std::endl;
	} catch (const std::exception& e) {
		std::cerr << "Unexpected error: " << e.what() << std::endl;
	}
	
	if (age >= 18) {
		std::cout << "You are an adult." << std::endl;
	} else {
		std::cout << "You are not an adult." << std::endl;
	}
	
	return 0;
}
```

### Error types in C++

```cpp
#include <stdexcept>
```

While there isn't one definitive list of all exception classes in the C++ standard library, here are some of the most important ones that reside in the `<stdexcept>` header:

**Base class:**
- `std::exception`: This is the base class for all standard exceptions. It provides methods like `what()` to retrieve the error message.

**Common exception classes:**
- `std::logic_error`: This indicates a programming error, suggesting something is wrong with the code's logic.
    - `std::domain_error`: A specific type of logic error where an argument is outside the domain of an operation.
    - `std::invalid_argument`: Another logic error due to an invalid argument passed to a function.
    - `std::length_error`: Indicates an error related to the length of a sequence or string.
    - `std::out_of_range`: Error due to an index or value being out of range.
- `std::runtime_error`: Occurs during program execution due to external factors like file I/O issues or resource exhaustion.
    - `std::range_error`: Error caused by invalid operations on iterators or ranges.
    - `std::overflow_error`: Occurs when an arithmetic operation results in an overflow or underflow.
    - `std::underflow_error`: Similar to `overflow_error`, but for underflow conditions.
- `std::future_error`: Indicates an error related to asynchronous operations using `std::future`.

**Less common but sometimes useful:**
- `std::bad_alloc`: Thrown when a memory allocation fails.
- `std::bad_typeid`: Thrown when a dynamic_cast operation fails due to type mismatch.
- `std::bad_exception`: Exception thrown by the exception handling mechanism itself.

**Additional notes:**
- Several other less common exception classes exist in the `<stdexcept>` header.
- You can create your own custom exception classes derived from `std::exception` for domain-specific needs.
- Choosing the right exception type depends on the specific error condition and context.

### Error codes in C/C++

While C++ doesn't have a central repository like a "list of all standard error codes," providing a comprehensive list requires understanding different sources of error codes:

**1. Standard C Library (`<cerrno>`):**

This header defines `errno`, a global variable holding the integer error code for the last system call that failed. Each code has a corresponding symbolic name defined in `<cerrno>`. Some common examples:
- `EACCES`: Permission denied
- `ENOENT`: No such file or directory
- `EINVAL`: Invalid argument
- `EAGAIN`: Resource temporarily unavailable

**2. POSIX Error Codes:**

Many systems implement POSIX standards, providing a wider range of error codes beyond core C library functions. These codes usually start with `E` and may not have corresponding symbolic names in `<cerrno>`.

**3. Standard C++ Library (`<stdexcept>`):**

Standard exceptions like `std::runtime_error` and its derived classes don't use specific codes but carry error messages through the `what()` method.

**4. Platform-Specific Error Codes:**

Different operating systems and libraries might have their own sets of error codes. These are platform-dependent and require consulting specific documentation.

**5. Custom Error Codes:**

Applications can define and use custom error codes for domain-specific errors.

**Here's a breakdown of key points:**

- There's no single "standard" list of error codes in C++.
- Understanding error codes involves multiple sources: `<cerrno>`, POSIX standards, library documentation, and potentially custom codes.
- Symbolic names can help readability, but not all codes have them.
- Choose the appropriate error code source based on the system call, library, or custom domain.

**Instead of seeking a complete list, focus on:**

- Identifying the relevant system call, library function, or error domain.
- Consulting the corresponding documentation for available error codes and their meanings.
- Using descriptive error messages to aid debugging and troubleshooting.

Assertions and exception handling are both methods for handling unexpected conditions in C++, but they serve different purposes and have distinct uses. Here's a breakdown:

## Assertions

- **Purpose:** Check for assumptions that should always be true in your code during development and debugging.
- **Mechanism:** Use the `assert` macro or function, which throws an `AssertionError` if the condition is false.
- **Benefits:**
    - Catch logic errors early on.
    - Provide informative error messages to help identify the issue.
    - Can be disabled in production builds for performance reasons.
- **Drawbacks:**
    - Not meant for handling runtime errors or external factors.
    - Add additional overhead to the code.

**Example:**

```cpp
#include <cassert>

void divide(int a, int b) {
	assert(b != 0); // Check for division by zero
	std::cout << a / b << std::endl;
}
```

## Exception Handling:

- **Purpose:** Handle unexpected occurrences during program execution, such as invalid input, file errors, network issues, etc.
- **Mechanism:** Use the `try-catch` block to define code that might throw exceptions and specify how to handle them.
- **Benefits:**
    - Makes code more robust and resilient to unexpected errors.
    - Separates error handling from normal program logic.
- **Drawbacks:**
    - Can impact performance and code readability if used excessively.
    - Choosing the right exceptions and catch blocks is crucial for proper error handling.

**Example:**

```cpp
#include <stdexcept>

void readFile(const std::string& filename) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		throw std::runtime_error("Failed to open file: " + filename);
	}
	// ... read file content
}

int main() {
	try {
		readFile("data.txt");
	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
	}
}
```

## Key Differences

- **Use:** Assertions for development-time checks, exceptions for runtime errors.
- **Scope:** Assertions often local to specific logic, exceptions can propagate through call stacks.
- **Disabling:** Assertions can be disabled, exceptions usually not.

## Choosing the Right Approach

- Use assertions to verify assumptions you expect to be always true.
- Use exceptions for unexpected errors beyond your code's control.
- Balance the use of both for robust and maintainable C++ code.