# Rule of Three/Five

From ChatGPT 3.5:

The "Rule of Three" and the "Rule of Five" are programming principles in C++ that relate to the management of resource ownership, particularly when building classes that handle resources like dynamic memory, file handles, or network connections. These rules are critical for ensuring that classes manage their resources correctly, avoid memory leaks, and prevent other resource mismanagement issues that can lead to bugs and inefficient software.

## Rule of Three

The Rule of Three states that if a class defines any of the following, it should probably explicitly define all three:

1. **Destructor** - Manages the cleanup and deallocation of resources when an object of the class is destroyed.
2. **Copy Constructor** - Manages how an object of the class is copied. It is invoked when a new object is created from an existing object, copying the values of all fields.
3. **Copy Assignment Operator** - Manages how an object of the class is assigned from another object of the same class.

The necessity of this rule arises from the need to ensure that copies of objects handle resources such as dynamically allocated memory correctly. Without all three, a class might handle copies inappropriately, leading to issues like double deletions, memory leaks, and other unintended behaviors.

## Rule of Five

Introduced with C++11, which added support for move semantics, the Rule of Five extends the Rule of Three. It suggests that if a class defines any of the following, it should probably explicitly define all five:

1. **Destructor**
2. **Copy Constructor**
3. **Copy Assignment Operator**
4. **Move Constructor** - Efficiently transfers resources from a temporary (rvalue) object to a new object, leaving the temporary in a valid but unspecified state.
5. **Move Assignment Operator** - Efficiently transfers resources from a temporary object to an existing object, leaving the temporary in a valid but unspecified state.

The addition of move constructors and move assignment operators allows for optimization opportunities by enabling the "moving" of resources from one object to another, rather than copying. This is especially useful for managing resources in performance-critical applications, as it can significantly reduce unnecessary copying.

## When to Use These Rules

If your class manages resources that are not handled by simple scalar fields or does not rely exclusively on RAII (Resource Acquisition Is Initialization) objects for managing resources, you should consider applying these rules. They are particularly relevant for:

* Classes that directly manage memory through pointers.
* Classes that handle operating system resources such as file handles or network sockets.

Applying the Rule of Three or Rule of Five helps to ensure that your C++ classes manage their resources safely and efficiently, preventing common errors like resource leaks or invalid memory accesses.
