# Smart Pointers

## What?

Smart pointers refer to **unique pointers** and **shared pointers**.

```cpp
std::unique_ptr
std::shared_ptr
```

Raw pointers are the traditional C/C++ pointers:

```cpp
// Static
int a = 5;
int* ptr = &a;

// Dynamic
int* ptr;
// Initialize
ptr = new int(5);
// or...
*ptr = 5;
```

## Why?

The benefit of smart pointers over `new` and `delete` is they manage deletion for you, so you don't have memory leaks. They automatically delete and free up the memory after the last owner goes out of scope.

## How do Smart Pointers Work?

Smart pointers keep track of ownership. When you pass a smart pointer to another smart pointer variable, the receiving variable/scope will either become *the* new owner (in the case of `std::unique_ptr`), or *a* new owner (in the case of `std::shared_ptr`).

When all owners have gone out of scope, the smart pointer is automatically deleted and its memory is freed.

## When NOT to Use Smart Pointers

If you're going to allocate memory statically by instantiating the object instance at the same time you create the pointer, just use a regular raw pointer.

```cpp
int a = 5;
int* ptr = &a;
```

## When to Use Smart Pointers

Smart pointers are only relevant for dynamic memory allocation. They replace `new` and `delete`.

Dynamic memory allocation is usually needed when:
- You don't know how many instances of an object you will need, so you create a container (like a `std::vector`) of pointers and add objects to it as needed
- You need a container of objects, and there is a separate function that instantiates the objects which you need to pass the container to, and the container needs to survive the function exiting 

### When to Use `std::unique_ptr`

Use `std::unique_ptr` when the variable that initially creates it is the only one you intend to modify the underlying value. Only owners can modify, so if you're only going to pass the `std::unique_ptr` to things that will not modify it, you should use a `std::unique_ptr`.

Ways that ownership gets transferred:
- A `std::unique_ptr` is copied from one variable to another
- A `std::unique_ptr` moves into a different scope
	- Passed to a function argument of type `std::unique_ptr`

Try to use `std::unique_ptr` as much as possible because:
- It handles deletion automatically
- It has a single defined owner
- It is the most resource efficient

Use `std::unique_ptr` if:
- The pointer will only be used by consumers
- And, you don't need to pass ownership of the pointer

Alternatively, if you want to pass ownership, and you don't need access to the pointer after it is passed to the new owner, you can use a `std::unique_ptr`.

Don't try to access it in the original scope after you pass ownership.

### How to Pass a `std::unique_ptr`

If you want to pass ownership, pass the `std::unique_ptr` directly to a function with argument type `std::unique_ptr`. The receiver will now be the new owner, and the pointer will be deleted when the new owner goes out of scope. Only do this if you don't need the original owner to access the pointer after the function ends.

```cpp
// Pass to a new owner

std::unique_ptr<int> a;
a = std::make_unique<int>(5);

// Pass ownership of `a` to `b`.
void func(std::unique_ptr<int> b) {
	std::cout << *b << std::endl;
}
func(a);
```

If you don't want to pass ownership (pass to a consumer), either:
- **Read/Write:** Pass the raw pointer managed by the `std::unique_ptr` using `my_ptr.get()`
- **Read/Write:** Pass a reference to the unique pointer (`std::unique_ptr<>&`)
- **Read Only:** Pass a `const &` of the `std::unique_ptr` (`const std::unique_ptr<>&`)

```cpp
// Pass to a consumer
// and retain ownership in original scope.

// 2 options (depends on company style).

// Option 1:
	// This will prevent modification while
	// preserving ownership semantics. This
	// is pure read-only.
void func(const std::unique_ptr<int>& b) {
	std::cout << *b << std::endl;
}
func(a);

// Option 2:
	// This will allow modification, so only
	// use it if you know that the original owner
	// will not delete the pointer until after
	// the function exits.
void func(int* b) {
	std::cout << *b << std::endl;
}
int* a_raw = a.get();
func(a_raw);
```

### Adding a `std::unique_ptr` to a STL Container

If you add a `std::unique_ptr` to a container using `emplace()`, `emplace_back()`, `push_back()` or similar, ownership will be transferred to the container. The pointer will survive the emplace/push function exiting. See source code of these methods for details.