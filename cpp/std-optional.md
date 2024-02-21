Conditional / Ternary operator:

```cpp
# Calls `Process()` on member_ if member_ has
# a value, otherwise returns false.
return member_ ? member_.value().Process() : false;
```