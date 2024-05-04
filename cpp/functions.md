# Functions

## Default Parameters

In a function with default parameters, the arguments are optional.

```cpp
void myfunction(int a=5, int b=1) {
}
```

You can mix required and optional arguments, but the optional ones must come after the required ones.

```cpp
void myfunction(int a, int b=1) {
}
```

## Function Overloading (Polymorphism)

```cpp
// The function that matches the # arguments, and data types of
// the arguments provided, is the one that's called.
int myfunction(int a=5, int b=1) {
	return a+b;
}
double myfunction(double a=5.1, double b=4.4) {
	return a+b;
}
```

Passing Pointers

```cpp
void increment(int &number) {
}
```

Constant Member Functions of a Class:

From Codeium:

> **Const Member Function**:
>
> * A const member function is a member function that is declared with the `const` keyword at the end of the function signature.
> * It promises not to modify the state of the object on which it is called.
> * Inside a const member function, you cannot modify non-mutable data members, call non-const member functions, or modify the object itself.
> * It is a good practice to mark member functions as const if they do not modify the object's state.

> Correct, the `const` keyword at the end of a member function declaration has no direct impact on whether the arguments passed to the function are modified.
>
> The `const` keyword at the end of a member function declaration specifically relates to the object on which the member function is called. It indicates that the member function does not modify the state of the object itself.
