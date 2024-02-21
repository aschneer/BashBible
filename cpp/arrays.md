# Arrays

```cpp
int myarray[6];
int myarray[6] = {4,8,15,16,23,42};
int myarray[] = {4,8,15,16,23,42};
```

# Vectors



# `std::initializer_list`

Allows for defining read-only, fixed-sized arrays in-place.

```cpp
{1, 2, 3, 4, 5}
```

Use for...

```cpp
// Initializing an array in place.
std::vector<int> numbers = {1, 2, 3, 4, 5};

// Passing an array to a function.
void processNumbers(std::initializer_list<int> nums) {
	for (int num : nums) {
		// Process each number
	}
}
processNumbers({10, 20, 30, 40, 50});
```