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

From [StackOverflow](https://stackoverflow.com/a/42152196):

> Simplified basic C++ namespace rules are:
>
> * You can access anything in parent namespace path without specifying namespace.
> * You can access anything in child namespace path by specifying only relative path.
> * Everything else requires full namespace specifications.

## "using" Keyword

Bring in a specific object from a different namespace, so you can refer to it directly.

```cpp
using my::space::MyClass;
// You can now refer to MyClass directly by name
// instead of using the full namespace prefix.
```

Bring in an entire namespace.

```cpp
using namespace std;
// You can now use anything from the `std` namespace
// directly by name without using the `std::` prefix.
```
