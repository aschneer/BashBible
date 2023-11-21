# Functions

```cpp
// Function with default parameters.
// Arguments become optional.
void myfunction(int a=5, int b=1) {
}
```

Function Overloading (Polymorphism)

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