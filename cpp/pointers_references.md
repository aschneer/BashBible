# Pointers

```cpp
int* mypointer;
mypointer = &myvariable;
*mypointer = 23;
```

## Pointers and Arrays

```cpp
int myarray[6] = {4,8,15,16,23,42};
int* mypointer;
mypointer = myarray;
*mypointer = 10;  // Updates first element of array
mypointer++;  // Increments pointer to second element of array
*mypointer = 20;  // Updates second element of array

// Other methods to increment array pointer:
// Method 1
mypointer = &myarray[2];
*mypointer = 30;
// Method 2
mypointer = myarray + 3;
*mypointer = 40;
// Method 3
mypointer = myarray;
*(mypointer + 4) = 50;
```

## Pointers/References to Constants

```cpp
const int* pointer = &variable;
*pointer = 20;  // Won't work - constant, can't modify

// Multiple ways to declare:
int* pointer;  // non-constant pointer to non-constant integer
const int* pointer;  // non-constant pointer to constant integer
int* const pointer;  // constant pointer to non-constant integer
const int* const pointer;  // constant pointer to constant integer
const int& ref;  // non-constant reference to constant integer
```

## References

References are different than pointers.

```cpp
// The following are equivalent.
int& x = 5;
int &x = 5;

// After a reference is defined, it can be used
// exactly as a normal variable without any special
// dereferencing or syntax (like a pointer would).
// Any changes to the variable will update the same
// memory location.
x = 3;

// This function accepts as input either a normal integer
// (like `int a`) or a reference to an integer (like x above).
// Regardless of what is passed in, the function will receive
// the reference to that integer. This function will also
// return a reference to an integer.
int& func(int& input) { return 3; }
int& func(int &input) { return 3; } // equivalent

// Running the function, you treat the input and output
// like normal integers, no special syntax required.
// Just know you are receiving a reference to the same
// integer in memory used in the function.
int result = func(x);

// So basically, there's nothing special you have to do
// to use references. You just have to define the data type
// in a function's input or output as a reference data type.
```

Example:

```cpp
int x = 5;
int& y = x;
y = 10; // This will modify the value of x to be 10.
// Both x and y refer to the same memory location.
```

## Functions - Passing by Value vs. by Reference:

```cpp
// Pass by Value:
void func(int i) { return 0; }  // Function declaration.
int num = 4;
func(num);  // Function call

// Pass by Reference:
void func(int& i) { return 0; }  // Function declaration.
void func(int &i) { return 0; }  // Function declaration (equivalent).
int num = 4;
func(num);  // Function call
  // NO DIFFERENCE IN HOW YOU PASS THE ARGUMENT BY VALUE VS. BY REFERENCE!
  // ONLY DIFFERENCE IS IN THE FUNCTION DEFINITION.

// Pass Pointer:
void func(int* i) { return 0; }  // Function declaration.
void func(int *i) { return 0; }  // Function declaration (equivalent).
int num = 4;
int* num_ptr = &num;
func(num_ptr);  // Function call
func(&num);  // Function call - should also work
```
