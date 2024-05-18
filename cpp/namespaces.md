# Namespaces

## Namespace Alias

```cpp
// Use type name "json" instead of "nlohmann::json"
using json = nlohmann::json;
```

## Global Namespace

A `::` with nothing in front of it means starting at the global namespace.

```cpp
// Google Test example:
class MyClassTest : public ::testing::Test {
};
```

## Nested Namespaces

A child namespace has access to everything in its parent/ancestor namespaces.

```cpp
my::parent {

class ParentNameClass {
};

}

my::parent::child {

ParentNameClass parent_instance;

}
```

## "using" Keyword

```cpp
using my::space::MyClass;
// You can now refer to MyClass directly by name
// instead of using the full namespace prefix.
```

```cpp
using namespace std;
// You can now use anything from the `std` namespace
// directly by name without using the `std::` prefix.
```
