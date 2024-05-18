# Optional Parameters

## std::optional

Declare optional parameter:

```cpp
#include <optional>
std::optional<int> opt_int;
```

Set value:

```cpp
opt_int = 3;
```

Check if it has a value:

```cpp
opt_int.has_value();
```

Get value:

```cpp
opt_int.value();
// Throws std::bad_optional_access if value not present
```

Get value, and set a default if value not present:

```cpp
opt_int.value_or(0);
```

Reset optional parameter to empty:

```cpp
opt_int.reset();
// or
opt_int = std::nullopt;
```

## Conditional / Ternary Operator

```cpp
// Calls `Process()` on member_ if member_ has
// a value, otherwise returns false.
return member_ ? member_.value().Process() : false;
```
