# Pointers

```cpp
int* mypointer;
mypointer = &myvariable;
*mypointer = 23;
```

Pointers and Arrays

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

Pointers to Constants

```cpp
const int* pointer = &variable;
*pointer = 20;  // Won't work - constant, can't modify

// Multiple ways to declare:
int* pointer;  // non-constant pointer to non-constant integer
const int* pointer;  // non-constant pointer to constant integer
int* const pointer;  // constant pointer to non-constant integer
const int* const pointer;  // constant pointer to constant integer
```