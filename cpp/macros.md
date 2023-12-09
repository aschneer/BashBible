# Macros

a.k.a. Pre-Processor Directives

## Macros with GFlags

How to declare GFlags macros in a header file, and include that header file in multiple source files without causing a duplicate macro definition compile error:

```cpp
// CORRECT WAY TO DO IT:
//
// In header file
DECLARE_bool(myflag);
// In source file
DEFINE_bool(myflag, true, "Explanation of what myflag does");

// WRONG WAY TO DO IT (WON'T WORK):
//
// The following DOES NOT WORK with GFlags macro definitions.
// If this is in the header file and gets included in multiple
// source files, it will still cause a duplicate macro def error.
#ifndef FLAGS_myflag
DEFINE_bool(myflag, true, "Explanation of what myflag does");
#endif

```