
# Scope

How do you make objects declared within a function persist after the function exits?

## Wrap it in a class (a shared context)

- Make the function and object variable members of the same class so they share scope
- Use the constructor to create the object and initialize its value in one shot

```cpp
class MyClass {
public:
	int var_;
	
	// Constructor
	MyClass(int val) : var_(val) {}
	
	// Function to modify member variable.
	void my_func() {
		var = 3;
	}
}
```

## Smart pointer method

- Create a smart pointer (`std::shared_ptr` or `std::unique_ptr`) outside the function scope
- Pass the smart pointer into the function
- Use either the `new` keyword or `std::make_shared()`/`std::make_unique()` to allocate memory and assign the new object to the pointer
	- `new` can only be used with a `std::unique_ptr`; with a `std::shared_ptr` you must use `std::make_shared`
- The new object will persist after the function exits

```cpp
// Example 1
void modifyPointer(std::unique_ptr<int>& my_pointer) {
	my_pointer.reset(new int(53));
}

// Example 2
void modifyPointer(std::unique_ptr<int>& my_pointer) {
	my_pointer = std::make_unique<int>(53);
}

// Example 3
void modifyPointer(std::shared_ptr<int>& my_pointer) {
	my_pointer = std::make_shared<int>(53);
}
```

## Double pointer, dynamic memory method

- Pass a double pointer into the function
- Change the value the double pointer points to, which is itself a pointer (address) where the object created inside the function will be stored
- No need to return anything from the function

```cpp
// Generated with Codeium
void modifyPointer(int** ptr) {
	int* newPtr = new int(42); // Dynamically allocate a new integer
	delete *ptr; // Deallocate the previous memory pointed by *ptr
	*ptr = newPtr; // Assign the new pointer value to *ptr
}
```

## `static` keyword method

- Declare the object in the function using the `static` keyword
- This will make the object persist after the function exits, but it will only be available within the scope of the function (i.e. if the function runs again)

```cpp
// `obj` will be available the next time
// the function runs, and it will retain its value.
void my_function() {
	static MyClass obj(3);
}
```